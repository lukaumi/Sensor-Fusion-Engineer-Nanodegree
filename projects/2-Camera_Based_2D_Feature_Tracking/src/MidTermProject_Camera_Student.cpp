/* INCLUDES FOR THIS PROJECT */
#include <cmath>
#include <fstream>
#include <getopt.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <signal.h>
#include <sstream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

// input argument options
static struct option options[] = {
    {InputArgumentKeyToString(InputArguments::DETECTOR_TYPE), required_argument, 0, static_cast<uint32_t>(InputArguments::DETECTOR_TYPE)},
    {InputArgumentKeyToString(InputArguments::DESCRIPTOR_TYPE), required_argument, 0, static_cast<uint32_t>(InputArguments::DESCRIPTOR_TYPE)},
    {InputArgumentKeyToString(InputArguments::DESCRIPTOR_CONTENT_TYPE), required_argument, 0, static_cast<uint32_t>(InputArguments::DESCRIPTOR_CONTENT_TYPE)},
    {InputArgumentKeyToString(InputArguments::MATCHER_TYPE), required_argument, 0, static_cast<uint32_t>(InputArguments::MATCHER_TYPE)},
    {InputArgumentKeyToString(InputArguments::SELECTOR_TYPE), required_argument, 0, static_cast<uint32_t>(InputArguments::SELECTOR_TYPE)},
    {InputArgumentKeyToString(InputArguments::VEHICLE_FOCUS_ONLY), no_argument, 0, static_cast<uint32_t>(InputArguments::VEHICLE_FOCUS_ONLY)},
    {InputArgumentKeyToString(InputArguments::VISUALIZE_RESULTS), no_argument, 0, static_cast<uint32_t>(InputArguments::VISUALIZE_RESULTS)},

    {nullptr, 0, nullptr, 0}};

// print the usage of this tool
static void usage(const char *binary)
{
    std::cout << "Usage: " << binary
              << " --detector_type [arg] --descriptor_type [arg] --descriptor_content_type [arg] --matcher_type [arg] --selector_type [arg] --vehicle_focus_only --visualize_results" << std::endl
              << "\t* detector_type arg from: SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT" << std::endl
              << "\t* descriptor_type arg from:  BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT" << std::endl
              << "\t* descriptor_content_type arg from: DES_BINARY, DES_HOG" << std::endl
              << "\t* matcher_type arg from: MAT_BF, MAT_FLANN" << std::endl
              << "\t* selector_type arg from: SEL_NN, SEL_KNN" << std::endl
              << "\t* vehicle_focus_only is optional and does not take any arguments" << std::endl
              << "\t* visualize_results is optional and does not take any arguments" << std::endl;
}

// signal handler
static void sigHandler(int signum)
{
    std::cout << "signal " << signum << " received" << std::endl;
    exit(signum);
}

/* MAIN PROGRAM */
int main(int argc, char *argv[])
{
    // set up signal handler
    struct sigaction sigact
    {
    };
    sigact.sa_handler = sigHandler;
    sigact.sa_flags = 0;
    sigemptyset(&sigact.sa_mask);
    sigaction(SIGINT, &sigact, nullptr);
    sigaction(SIGTERM, &sigact, nullptr);
    sigaction(SIGQUIT, &sigact, nullptr);

    /* INPUT ARGUMENTS */
    std::string detectorType = "";
    std::string descriptorType = "";
    std::string descriptorContentType = "";
    std::string matcherType = "";
    std::string selectorType = "";
    bool bFocusOnVehicle = false;
    bool bVis = false;

    int32_t opt;
    while ((opt = getopt_long_only(argc, argv, "", options, nullptr)) != -1)
    {
        switch (opt)
        {
        case static_cast<uint32_t>(InputArguments::DETECTOR_TYPE):
            detectorType = optarg;
            break;
        case static_cast<uint32_t>(InputArguments::DESCRIPTOR_TYPE):
            descriptorType = optarg;
            break;
        case static_cast<uint32_t>(InputArguments::DESCRIPTOR_CONTENT_TYPE):
            descriptorContentType = optarg;
            break;
        case static_cast<uint32_t>(InputArguments::MATCHER_TYPE):
            matcherType = optarg;
            break;
        case static_cast<uint32_t>(InputArguments::SELECTOR_TYPE):
            selectorType = optarg;
            break;
        case static_cast<uint32_t>(InputArguments::VEHICLE_FOCUS_ONLY):
            bFocusOnVehicle = true;
            break;
        case static_cast<uint32_t>(InputArguments::VISUALIZE_RESULTS):
            bVis = true;
            break;
        default:
            usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (optind < argc || argc < 2 ||
        detectorType == "" || descriptorType == "" || descriptorContentType == "" || matcherType == "" || selectorType == "")
    {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    if ((descriptorType == DescriptorKeyToString(Descriptors::AKAZE)) && (detectorType != DetectorKeyToString(Detectors::AKAZE)))
    {
        std::cout << "WARNING: AKAZE descriptors can only be used with KAZE or AKAZE keypoints!" << std::endl;
        detectorType = DetectorKeyToString(Detectors::AKAZE);
        std::cout << "Switching detector to " << detectorType << std::endl;
    }

    std::cout << std::endl
              << "detector type argument: " << detectorType << std::endl
              << "descriptor type argument: " << descriptorType << std::endl
              << "descriptor content type argument: " << descriptorContentType << std::endl
              << "matcher type argument: " << matcherType << std::endl
              << "selector type argument: " << selectorType << std::endl
              << "vehicle focus only: " << std::boolalpha << bFocusOnVehicle << std::endl
              << "visualize results: " << std::boolalpha << bVis << std::endl
              << std::endl;

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    const std::string dataPath = "../";

    // camera
    const std::string imgBasePath = dataPath + "images/";
    const std::string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    const std::string imgFileType = ".png";
    const int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    const int imgEndIndex = 9;   // last file index to load
    const int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    const int dataBufferSizeLimit = 2; // no. of images which are held in memory (ring buffer) at the same time
    std::vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time

    std::string resultMP7 = "| ";
    std::string resultMP8 = "";
    uint32_t resultMP8Sum = 0;

    double detectorTimeResult = 0.0;
    double descriptorTimeResult = 0.0;
    uint8_t imgCount = 0;

    /* MAIN LOOP OVER ALL IMAGES */
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        std::ostringstream imgNumber;
        imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;
        std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSizeLimit

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        std::size_t currentBufferSize = dataBuffer.size();
        if (currentBufferSize == dataBufferSizeLimit)
        {
            // remove first frame element from dataBuffer
            dataBuffer.erase(dataBuffer.begin());
        }
        else if (currentBufferSize > dataBufferSizeLimit)
        {
            // there can only be `dataBufferSizeLimit` number of elements in dataBuffer
            std::cout << "MP.1 : ERROR: dataBuffer number of elements!" << std::endl;
            return EXIT_FAILURE;
        }
        // insert new frame at the end
        dataBuffer.push_back(frame);

        //// EOF STUDENT ASSIGNMENT - DONE
        std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType == DetectorKeyToString(Detectors::SHITOMASI))
        {
            detectorTimeResult += detKeypointsShiTomasi(keypoints, imgGray, bVis);
        }
        else if (detectorType == DetectorKeyToString(Detectors::HARRIS))
        {
            detectorTimeResult += detKeypointsHarris(keypoints, imgGray, bVis);
        }
        else if (detectorType == DetectorKeyToString(Detectors::FAST) ||
                 detectorType == DetectorKeyToString(Detectors::BRISK) ||
                 detectorType == DetectorKeyToString(Detectors::ORB) ||
                 detectorType == DetectorKeyToString(Detectors::AKAZE) ||
                 detectorType == DetectorKeyToString(Detectors::SIFT))
        {
            detectorTimeResult += detKeypointsModern(keypoints, imgGray, detectorType, bVis);
        }
        else
        {
            std::cout << "MP.2 : ERROR: Invalid detector type: " << detectorType << std::endl;
            usage(argv[0]);
            return EXIT_FAILURE;
        }
        //// EOF STUDENT ASSIGNMENT - DONE

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            // use erase-remove
            keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(),
                                           [&vehicleRect](const cv::KeyPoint &keypoint)
                                           { return !vehicleRect.contains(keypoint.pt); }),
                            keypoints.end());

            resultMP7 += std::to_string(keypoints.size()) + " | ";
        }

        //// EOF STUDENT ASSIGNMENT - DONE

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType == "SHITOMASI")
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            std::cout << "NOTE: Keypoints have been limited!" << std::endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        std::cout << "#2 : DETECT KEYPOINTS done" << std::endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        if (descriptorType == DescriptorKeyToString(Descriptors::BRISK) ||
            descriptorType == DescriptorKeyToString(Descriptors::BRIEF) ||
            descriptorType == DescriptorKeyToString(Descriptors::ORB) ||
            descriptorType == DescriptorKeyToString(Descriptors::FREAK) ||
            descriptorType == DescriptorKeyToString(Descriptors::AKAZE) ||
            descriptorType == DescriptorKeyToString(Descriptors::SIFT))
        {
            descriptorTimeResult += descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        }
        else
        {
            std::cout << "MP.4 : ERROR: Invalid descriptor type: " << descriptorType << std::endl;
            usage(argv[0]);
            return EXIT_FAILURE;
        }
        //// EOF STUDENT ASSIGNMENT - DONE

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        std::cout << "#3 : EXTRACT DESCRIPTORS done" << std::endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {
            /* MATCH KEYPOINT DESCRIPTORS */

            std::vector<cv::DMatch> matches;

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            if (descriptorContentType != DescriptorContentTypeKeyToString(DescriptorsContentTypes::DES_BINARY) &&
                descriptorContentType != DescriptorContentTypeKeyToString(DescriptorsContentTypes::DES_HOG))
            {
                std::cout << "MP.5 : ERROR: Invalid descriptor content type: " << descriptorContentType << std::endl;
                usage(argv[0]);
                return EXIT_FAILURE;
            }
            if (matcherType != MatcherKeyToString(Matchers::MAT_BF) &&
                matcherType != MatcherKeyToString(Matchers::MAT_FLANN))
            {
                std::cout << "MP.5 : ERROR: Invalid matcher type: " << matcherType << std::endl;
                usage(argv[0]);
                return EXIT_FAILURE;
            }
            if (selectorType != SelectorKeyToString(Selectors::SEL_NN) &&
                selectorType != SelectorKeyToString(Selectors::SEL_KNN))
            {
                std::cout << "MP.5 : ERROR: Invalid selector type: " << selectorType << std::endl;
                usage(argv[0]);
                return EXIT_FAILURE;
            }

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorContentType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT - DONE

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            std::cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;

            resultMP8Sum += matches.size();
            resultMP8 += std::to_string(matches.size()) + ",";

            // visualize matches between current and previous image
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                std::string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                std::cout << "Press key to continue to next image" << std::endl
                          << std::endl;
                cv::waitKey(0); // wait for key to be pressed
            }
        }

        imgCount++;
    } // eof loop over all images

    if (bFocusOnVehicle)
    {
        std::cout << "Result MP.7 for " << detectorType << ": " << resultMP7 << std::endl;
    }

    resultMP8.pop_back(); // remove last comma
    std::cout << "Result MP.8 for " << detectorType << " / " << descriptorType << " combination" << std::endl
              << "    " << resultMP8 << std::endl
              << "    average: " << (resultMP8Sum / 1.0) / imgCount << std::endl;

    std::cout << "Result MP.9 [ms]" << std::endl
              << "    detector time: " << 1000 * (detectorTimeResult / imgCount) / 1.0 << std::endl
              << "    descriptor time: " << 1000 * (descriptorTimeResult / imgCount) / 1.0 << std::endl;

    return 0;
}
