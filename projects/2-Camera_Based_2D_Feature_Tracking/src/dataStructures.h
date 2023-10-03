#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>

// represents the available sensor information at the same time instance
struct DataFrame
{
    cv::Mat cameraImg; // camera image

    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors;                 // keypoint descriptors
    std::vector<cv::DMatch> kptMatches;  // keypoint matches between previous and current frame
};

enum class InputArguments : uint32_t
{
    DETECTOR_TYPE = 0,
    DESCRIPTOR_TYPE,
    DESCRIPTOR_CONTENT_TYPE,
    MATCHER_TYPE,
    SELECTOR_TYPE,
    VEHICLE_FOCUS_ONLY,
    VISUALIZE_RESULTS
};

// static instead of constexpr because -std=c++11
static const char *InputArgumentKeyToString(InputArguments key) noexcept
{
    switch (key)
    {
    case InputArguments::DETECTOR_TYPE:
        return "detector_type";
    case InputArguments::DESCRIPTOR_TYPE:
        return "descriptor_type";
    case InputArguments::DESCRIPTOR_CONTENT_TYPE:
        return "descriptor_content_type";
    case InputArguments::MATCHER_TYPE:
        return "matcher_type";
    case InputArguments::SELECTOR_TYPE:
        return "selector_type";
    case InputArguments::VEHICLE_FOCUS_ONLY:
        return "vehicle_focus_only";
    case InputArguments::VISUALIZE_RESULTS:
        return "visualize_results";
    default:
        return "";
    }
};

enum class Detectors : uint32_t
{
    SHITOMASI = 0,
    HARRIS,
    FAST,
    BRISK,
    ORB,
    AKAZE,
    SIFT
};

static const char *DetectorKeyToString(Detectors key) noexcept
{
    switch (key)
    {
    case Detectors::SHITOMASI:
        return "SHITOMASI";
    case Detectors::HARRIS:
        return "HARRIS";
    case Detectors::FAST:
        return "FAST";
    case Detectors::BRISK:
        return "BRISK";
    case Detectors::ORB:
        return "ORB";
    case Detectors::AKAZE:
        return "AKAZE";
    case Detectors::SIFT:
        return "SIFT";
    default:
        return "";
    }
};

enum class Descriptors : uint32_t
{
    BRISK = 0,
    BRIEF,
    ORB,
    FREAK,
    AKAZE,
    SIFT
};

static const char *DescriptorKeyToString(Descriptors key) noexcept
{
    switch (key)
    {
    case Descriptors::BRISK:
        return "BRISK";
    case Descriptors::BRIEF:
        return "BRIEF";
    case Descriptors::ORB:
        return "ORB";
    case Descriptors::FREAK:
        return "FREAK";
    case Descriptors::AKAZE:
        return "AKAZE";
    case Descriptors::SIFT:
        return "SIFT";
    default:
        return "";
    }
};

enum class DescriptorsContentTypes : uint32_t
{
    DES_BINARY = 0,
    DES_HOG
};

static const char *DescriptorContentTypeKeyToString(DescriptorsContentTypes key) noexcept
{
    switch (key)
    {
    case DescriptorsContentTypes::DES_BINARY:
        return "DES_BINARY";
    case DescriptorsContentTypes::DES_HOG:
        return "DES_HOG";
    default:
        return "";
    }
};

enum class Matchers : uint32_t
{
    MAT_BF = 0,
    MAT_FLANN
};

static const char *MatcherKeyToString(Matchers key) noexcept
{
    switch (key)
    {
    case Matchers::MAT_BF:
        return "MAT_BF";
    case Matchers::MAT_FLANN:
        return "MAT_FLANN";
    default:
        return "";
    }
};

enum class Selectors : uint32_t
{
    SEL_NN = 0,
    SEL_KNN
};

static const char *SelectorKeyToString(Selectors key) noexcept
{
    switch (key)
    {
    case Selectors::SEL_NN:
        return "SEL_NN";
    case Selectors::SEL_KNN:
        return "SEL_KNN";
    default:
        return "";
    }
};

#endif /* dataStructures_h */
