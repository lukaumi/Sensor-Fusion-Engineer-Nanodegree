#include <numeric>
#include "matching2D.hpp"

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource,
                      std::vector<cv::KeyPoint> &kPtsRef,
                      cv::Mat &descSource,
                      cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches,
                      std::string descriptorContentType,
                      std::string matcherType,
                      std::string selectorType)
{
    // configure matcher
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType == MatcherKeyToString(Matchers::MAT_BF))
    {
        const bool crossCheck = false;
        const int normType =
            descriptorContentType == DescriptorContentTypeKeyToString(DescriptorsContentTypes::DES_BINARY)
                ? cv::NORM_HAMMING
                : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
        // std::cout << "BF matching cross-check = " << crossCheck << std::endl;
    }
    else if (matcherType == MatcherKeyToString(Matchers::MAT_FLANN))
    {
        // OpenCV bug workaround: convert binary descriptors to floating point due to a bug in current OpenCV implementation
        if (descSource.type() != CV_32F)
        {
            descSource.convertTo(descSource, CV_32F);
        }
        if (descRef.type() != CV_32F)
        {
            descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }
    else
    {
        throw std::invalid_argument(std::string("Invalid matcher type value: " + matcherType));
    }

    // perform matching task
    if (selectorType == SelectorKeyToString(Selectors::SEL_NN))
    {
        // nearest neighbor (best match)
        matcher->match(descSource, descRef, matches); // finds the best match for each descriptor in desc1
    }
    else if (selectorType == SelectorKeyToString(Selectors::SEL_KNN))
    {
        // k nearest neighbors (k=2)
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher->knnMatch(descSource, descRef, knnMatches, 2); // finds the 2 best matches

        // filter matches using descriptor distance ratio test
        const double minDescDistRatio = 0.8;
        for (auto it = knnMatches.begin(); it != knnMatches.end(); ++it)
        {
            if ((*it)[0].distance < (*it)[1].distance * minDescDistRatio)
            {
                matches.push_back((*it)[0]);
            }
        }
        // std::cout << "# keypoints removed = " << knnMatches.size() - matches.size() << std::endl;
    }
    else
    {
        throw std::invalid_argument(std::string("Invalid selector type value: " + selectorType));
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType == DescriptorKeyToString(Descriptors::BRISK))
    {
        const int threshold = 30;        // FAST/AGAST detection threshold score.
        const int octaves = 3;           // detection octaves (use 0 to do single scale)
        const float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType == DescriptorKeyToString(Descriptors::BRIEF))
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType == DescriptorKeyToString(Descriptors::ORB))
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType == DescriptorKeyToString(Descriptors::FREAK))
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType == DescriptorKeyToString(Descriptors::AKAZE))
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType == DescriptorKeyToString(Descriptors::SIFT))
    {
        extractor = cv::xfeatures2d::SIFT::create();
    }
    else
    {
        throw std::invalid_argument(std::string("Invalid descriptor type value: " + descriptorType));
    }

    // perform feature description
    const double timeStart = static_cast<double>(cv::getTickCount());
    extractor->compute(img, keypoints, descriptors);
    const double timeEnd = (static_cast<double>(cv::getTickCount()) - timeStart) / cv::getTickFrequency();
    // std::cout << descriptorType << " descriptor extraction in " << 1000 * timeEnd / 1.0 << " ms" << std::endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    const int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    const double maxOverlap = 0.0; // max. permissible overlap between two features in %
    const double minDistance = (1.0 - maxOverlap) * blockSize;
    const int maxCorners = img.rows * img.cols / std::max(1.0, minDistance); // max. num. of keypoints

    const double qualityLevel = 0.01; // minimal accepted quality of image corners
    const double k = 0.04;

    // Apply corner detection
    const double timeStart = static_cast<double>(cv::getTickCount());

    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    const double timeEnd = (static_cast<double>(cv::getTickCount()) - timeStart) / cv::getTickFrequency();
    // std::cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * timeEnd / 1.0 << " ms" << std::endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Shi-Tomasi Keypoint Detector Results";
        cv::namedWindow(windowName, 5);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Detector parameters
    const int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    const int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    const int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    const double k = 0.04;       // Harris parameter (see equation for details)

    const double timeStart = static_cast<double>(cv::getTickCount());

    // Detect Harris corners and normalize output
    cv::Mat dst, dstNorm, dstNormScaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dstNorm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dstNorm, dstNormScaled);

    // Locate local maxima in the Harris response matrix
    // and perform a non-maximum suppression (NMS) in a local neighborhood around each maximum

    // Look for prominent corners and instantiate keypoints
    const double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
    for (size_t j = 0; j < dstNorm.rows; j++)
    {
        for (size_t i = 0; i < dstNorm.cols; i++)
        {
            int response = static_cast<int>(dstNorm.at<float>(j, i));
            if (response > minResponse)
            { // only store points above a threshold

                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool bOverlap = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }
                if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        } // eof loop over cols
    }     // eof loop over rows

    const double timeEnd = (static_cast<double>(cv::getTickCount()) - timeStart) / cv::getTickFrequency();
    // std::cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * timeEnd / 1.0 << " ms" << std::endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Harris Keypoint Detector Results";
        cv::namedWindow(windowName, 5);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    const double timeStart = static_cast<double>(cv::getTickCount());

    cv::Ptr<cv::FeatureDetector> detector;

    if (detectorType == DetectorKeyToString(Detectors::FAST))
    {
        // difference between intensity of the central pixel and pixels of a circle around this pixel
        const int fastThreshold = 30;
        // perform non-maxima suppression on keypoints
        const bool fastNMS = true;
        // TYPE_9_16, TYPE_7_12, TYPE_5_8
        const cv::FastFeatureDetector::DetectorType fastType = cv::FastFeatureDetector::TYPE_9_16;

        detector = cv::FastFeatureDetector::create(fastThreshold, fastNMS, fastType);
    }
    else if (detectorType == DetectorKeyToString(Detectors::BRISK))
    {
        detector = cv::BRISK::create();
    }
    else if (detectorType == DetectorKeyToString(Detectors::ORB))
    {
        detector = cv::ORB::create();
    }
    else if (detectorType == DetectorKeyToString(Detectors::AKAZE))
    {
        detector = cv::AKAZE::create();
    }
    else if (detectorType == DetectorKeyToString(Detectors::SIFT))
    {
        detector = cv::xfeatures2d::SIFT::create();
    }
    else
    {
        throw std::invalid_argument(std::string("Invalid detector type value: " + detectorType));
    }

    detector->detect(img, keypoints);

    const double timeEnd = (static_cast<double>(cv::getTickCount()) - timeStart) / cv::getTickFrequency();
    // std::cout << detectorType << " detection with n=" << keypoints.size() << " keypoints in " << 1000 * timeEnd / 1.0 << " ms" << std::endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = detectorType + " Keypoint Detector Results";
        cv::namedWindow(windowName, 5);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}
