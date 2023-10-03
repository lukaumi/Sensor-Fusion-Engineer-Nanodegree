/* INCLUDES FOR THIS PROJECT */
#include <cmath>
#include <fstream>
#include <getopt.h>
#include <iomanip>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <vector>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "objectDetection2D.hpp"
#include "lidarData.hpp"
#include "camFusion.hpp"

// TODO: logging order steps fix

// input argument options
static struct option options[] = {
    {InputArgumentKeyToString(InputArguments::DETECTOR_TYPE), required_argument, 0, static_cast<uint32_t>(InputArguments::DETECTOR_TYPE)},
    {InputArgumentKeyToString(InputArguments::DESCRIPTOR_TYPE), required_argument, 0, static_cast<uint32_t>(InputArguments::DESCRIPTOR_TYPE)},
    {InputArgumentKeyToString(InputArguments::DESCRIPTOR_CONTENT_TYPE), required_argument, 0, static_cast<uint32_t>(InputArguments::DESCRIPTOR_CONTENT_TYPE)},
    {InputArgumentKeyToString(InputArguments::MATCHER_TYPE), required_argument, 0, static_cast<uint32_t>(InputArguments::MATCHER_TYPE)},
    {InputArgumentKeyToString(InputArguments::SELECTOR_TYPE), required_argument, 0, static_cast<uint32_t>(InputArguments::SELECTOR_TYPE)},
    {InputArgumentKeyToString(InputArguments::VISUALIZE_RESULTS), no_argument, 0, static_cast<uint32_t>(InputArguments::VISUALIZE_RESULTS)},

    {nullptr, 0, nullptr, 0}};

// print the usage of this tool
static void usage(const char *binary)
{
    std::cout << "Usage: " << binary
              << " --detector_type [arg] --descriptor_type [arg] --descriptor_content_type [arg] --matcher_type [arg] --selector_type [arg] --visualize_results" << std::endl
              << "\t* detector_type arg from: SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT" << std::endl
              << "\t* descriptor_type arg from:  BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT" << std::endl
              << "\t* descriptor_content_type arg from: DES_BINARY, DES_HOG" << std::endl
              << "\t* matcher_type arg from: MAT_BF, MAT_FLANN" << std::endl
              << "\t* selector_type arg from: SEL_NN, SEL_KNN" << std::endl
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
              << "visualize results: " << std::boolalpha << bVis << std::endl
              << std::endl;

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    std::string dataPath = "../";

    // camera
    std::string imgBasePath = dataPath + "images/";
    std::string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color
    std::string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;  // last file index to load
    int imgStepWidth = 1;
    int imgFillWidth = 4; // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    // wget https://pjreddie.com/media/files/yolov3.weights "https://pjreddie.com/media/files/yolov3.weights"
    std::string yoloBasePath = dataPath + "dat/yolo/";
    std::string yoloClassesFile = yoloBasePath + "coco.names";
    std::string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    std::string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    std::string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    std::string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3, 4, cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4, 4, cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4, 4, cv::DataType<double>::type);        // rotation matrix and translation vector

    RT.at<double>(0, 0) = 7.533745e-03;
    RT.at<double>(0, 1) = -9.999714e-01;
    RT.at<double>(0, 2) = -6.166020e-04;
    RT.at<double>(0, 3) = -4.069766e-03;
    RT.at<double>(1, 0) = 1.480249e-02;
    RT.at<double>(1, 1) = 7.280733e-04;
    RT.at<double>(1, 2) = -9.998902e-01;
    RT.at<double>(1, 3) = -7.631618e-02;
    RT.at<double>(2, 0) = 9.998621e-01;
    RT.at<double>(2, 1) = 7.523790e-03;
    RT.at<double>(2, 2) = 1.480755e-02;
    RT.at<double>(2, 3) = -2.717806e-01;
    RT.at<double>(3, 0) = 0.0;
    RT.at<double>(3, 1) = 0.0;
    RT.at<double>(3, 2) = 0.0;
    RT.at<double>(3, 3) = 1.0;

    R_rect_00.at<double>(0, 0) = 9.999239e-01;
    R_rect_00.at<double>(0, 1) = 9.837760e-03;
    R_rect_00.at<double>(0, 2) = -7.445048e-03;
    R_rect_00.at<double>(0, 3) = 0.0;
    R_rect_00.at<double>(1, 0) = -9.869795e-03;
    R_rect_00.at<double>(1, 1) = 9.999421e-01;
    R_rect_00.at<double>(1, 2) = -4.278459e-03;
    R_rect_00.at<double>(1, 3) = 0.0;
    R_rect_00.at<double>(2, 0) = 7.402527e-03;
    R_rect_00.at<double>(2, 1) = 4.351614e-03;
    R_rect_00.at<double>(2, 2) = 9.999631e-01;
    R_rect_00.at<double>(2, 3) = 0.0;
    R_rect_00.at<double>(3, 0) = 0;
    R_rect_00.at<double>(3, 1) = 0;
    R_rect_00.at<double>(3, 2) = 0;
    R_rect_00.at<double>(3, 3) = 1;

    P_rect_00.at<double>(0, 0) = 7.215377e+02;
    P_rect_00.at<double>(0, 1) = 0.000000e+00;
    P_rect_00.at<double>(0, 2) = 6.095593e+02;
    P_rect_00.at<double>(0, 3) = 0.000000e+00;
    P_rect_00.at<double>(1, 0) = 0.000000e+00;
    P_rect_00.at<double>(1, 1) = 7.215377e+02;
    P_rect_00.at<double>(1, 2) = 1.728540e+02;
    P_rect_00.at<double>(1, 3) = 0.000000e+00;
    P_rect_00.at<double>(2, 0) = 0.000000e+00;
    P_rect_00.at<double>(2, 1) = 0.000000e+00;
    P_rect_00.at<double>(2, 2) = 1.000000e+00;
    P_rect_00.at<double>(2, 3) = 0.000000e+00;

    // misc
    double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    const int dataBufferSizeLimit = 2;            // no. of images which are held in memory (ring buffer) at the same time
    std::vector<DataFrame> dataBuffer;            // list of data frames which are held in memory at the same time

    std::string resultLidarTTC = "| ";
    std::string resultCameraTTC = "| ";

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex += imgStepWidth)
    {
        std::cout << std::endl
                  << "Img #" << imgIndex << std::endl;
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        std::ostringstream imgNumber;
        imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;
        std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file
        cv::Mat img = cv::imread(imgFullFilename);

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = img;
        std::size_t currentBufferSize = dataBuffer.size();
        if (currentBufferSize == dataBufferSizeLimit)
        {
            // remove first frame element from dataBuffer
            dataBuffer.erase(dataBuffer.begin());
        }
        else if (currentBufferSize > dataBufferSizeLimit)
        {
            // there can only be `dataBufferSizeLimit` number of elements in dataBuffer
            std::cout << "#1 : ERROR: dataBuffer number of elements!" << std::endl;
            return EXIT_FAILURE;
        }
        // insert new frame at the end
        dataBuffer.push_back(frame);

        std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;

        /* DETECT & CLASSIFY OBJECTS */

        float confThreshold = 0.2;
        float nmsThreshold = 0.4;
        detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, confThreshold, nmsThreshold,
                      yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, bVis);

        std::cout << "#2 : DETECT & CLASSIFY OBJECTS done" << std::endl;

        /* CROP LIDAR POINTS */

        // load 3D Lidar points from file
        std::string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
        std::vector<LidarPoint> lidarPoints;
        loadLidarFromFile(lidarPoints, lidarFullFilename);

        // remove Lidar points based on distance properties
        float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
        cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);

        (dataBuffer.end() - 1)->lidarPoints = lidarPoints;

        std::cout << "#3 : CROP LIDAR POINTS done" << std::endl;

        /* CLUSTER LIDAR POINT CLOUD */

        // associate Lidar points with camera-based ROI
        float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
        clusterLidarWithROI((dataBuffer.end() - 1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT);

        // Visualize 3D objects
        if (bVis)
        {
            show3DObjects((dataBuffer.end() - 1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), true);
        }

        std::cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << std::endl;

        /* DETECT IMAGE KEYPOINTS */

        // convert current image to grayscale
        cv::Mat imgGray;
        cv::cvtColor((dataBuffer.end() - 1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);

        // extract 2D keypoints from current image
        std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //// string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
        if (detectorType == DetectorKeyToString(Detectors::SHITOMASI))
        {
            detKeypointsShiTomasi(keypoints, imgGray, bVis);
        }
        else if (detectorType == DetectorKeyToString(Detectors::HARRIS))
        {
            detKeypointsHarris(keypoints, imgGray, bVis);
        }
        else if (detectorType == DetectorKeyToString(Detectors::FAST) ||
                 detectorType == DetectorKeyToString(Detectors::BRISK) ||
                 detectorType == DetectorKeyToString(Detectors::ORB) ||
                 detectorType == DetectorKeyToString(Detectors::AKAZE) ||
                 detectorType == DetectorKeyToString(Detectors::SIFT))
        {
            detKeypointsModern(keypoints, imgGray, detectorType, bVis);
        }
        else
        {
            std::cout << "#5 : ERROR: Invalid detector type: " << detectorType << std::endl;
            usage(argv[0]);
            return EXIT_FAILURE;
        }

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

        std::cout << "#5 : DETECT KEYPOINTS done" << std::endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        if (descriptorType == DescriptorKeyToString(Descriptors::BRISK) ||
            descriptorType == DescriptorKeyToString(Descriptors::BRIEF) ||
            descriptorType == DescriptorKeyToString(Descriptors::ORB) ||
            descriptorType == DescriptorKeyToString(Descriptors::FREAK) ||
            descriptorType == DescriptorKeyToString(Descriptors::AKAZE) ||
            descriptorType == DescriptorKeyToString(Descriptors::SIFT))
        {
            descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        }
        else
        {
            std::cout << "#6 : ERROR: Invalid descriptor type: " << descriptorType << std::endl;
            usage(argv[0]);
            return EXIT_FAILURE;
        }

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        std::cout << "#6 : EXTRACT DESCRIPTORS done" << std::endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {
            /* MATCH KEYPOINT DESCRIPTORS */

            std::vector<cv::DMatch> matches;
            //// perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            if (descriptorContentType != DescriptorContentTypeKeyToString(DescriptorsContentTypes::DES_BINARY) &&
                descriptorContentType != DescriptorContentTypeKeyToString(DescriptorsContentTypes::DES_HOG))
            {
                std::cout << "#7 : ERROR: Invalid descriptor content type: " << descriptorContentType << std::endl;
                usage(argv[0]);
                return EXIT_FAILURE;
            }
            if (matcherType != MatcherKeyToString(Matchers::MAT_BF) &&
                matcherType != MatcherKeyToString(Matchers::MAT_FLANN))
            {
                std::cout << "#7 : ERROR: Invalid matcher type: " << matcherType << std::endl;
                usage(argv[0]);
                return EXIT_FAILURE;
            }
            if (selectorType != SelectorKeyToString(Selectors::SEL_NN) &&
                selectorType != SelectorKeyToString(Selectors::SEL_KNN))
            {
                std::cout << "#7 : ERROR: Invalid selector type: " << selectorType << std::endl;
                usage(argv[0]);
                return EXIT_FAILURE;
            }

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorContentType, matcherType, selectorType);

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            std::cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;

            /* TRACK 3D OBJECT BOUNDING BOXES */

            //// STUDENT ASSIGNMENT
            //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
            std::map<int, int> bbBestMatches;
            matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end() - 2), *(dataBuffer.end() - 1));
            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->bbMatches = bbBestMatches;

            std::cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << std::endl;

            /* COMPUTE TTC ON OBJECT IN FRONT */

            // loop over all BB match pairs
            for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1)
            {
                // find bounding boxes associates with current match
                BoundingBox *prevBB, *currBB;
                for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2)
                {
                    if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB
                    {
                        currBB = &(*it2);
                    }
                }

                for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2)
                {
                    if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB
                    {
                        prevBB = &(*it2);
                    }
                }

                // compute TTC for current match
                if (currBB->lidarPoints.size() > 0 && prevBB->lidarPoints.size() > 0) // only compute TTC if we have Lidar points
                {
                    //// STUDENT ASSIGNMENT
                    //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                    double ttcLidar;
                    computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
                    resultLidarTTC += std::to_string(ttcLidar) + " | ";
                    //// EOF STUDENT ASSIGNMENT

                    //// STUDENT ASSIGNMENT
                    //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
                    //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
                    double ttcCamera;
                    clusterKptMatchesWithROI(*currBB, (dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->kptMatches);
                    computeTTCCamera((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints, currBB->kptMatches, sensorFrameRate, ttcCamera);
                    resultCameraTTC += std::to_string(ttcCamera) + " | ";
                    //// EOF STUDENT ASSIGNMENT

                    if (bVis)
                    {
                        cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
                        showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                        cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y), cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height), cv::Scalar(0, 255, 0), 2);

                        char str[200];
                        sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
                        putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255));

                        std::string windowName = "Final Results : TTC";
                        cv::namedWindow(windowName, 4);
                        cv::imshow(windowName, visImg);
                        std::cout << "Press key to continue to next frame" << std::endl;
                        cv::waitKey(0);
                    }

                } // eof TTC computation
            }     // eof loop over all BB matches
        }

    } // eof loop over all images

    std::cout << std::endl;
    std::cout << "Result Lidar TTC for " << detectorType << " / " << descriptorType << " combination" << std::endl
              << "    " << resultLidarTTC << std::endl;
    std::cout << std::endl;
    std::cout << "Result Camera TTC for " << detectorType << " / " << descriptorType << " combination" << std::endl
              << "    " << resultCameraTTC << std::endl;

    return 0;
}
