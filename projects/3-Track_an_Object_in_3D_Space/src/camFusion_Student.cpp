#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes,
                         std::vector<LidarPoint> &lidarPoints,
                         float shrinkFactor,
                         cv::Mat &P_rect_xx,
                         cv::Mat &R_rect_xx,
                         cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        std::vector<std::vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (std::vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/*
 * The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size.
 * However, you can make this function work for other sizes too.
 * For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
 */
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
        float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin < xw ? xwmin : xw;
            ywmin = ywmin < yw ? ywmin : yw;
            ywmax = ywmax > yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top < y ? top : y;
            left = left < x ? left : x;
            bottom = bottom > y ? bottom : y;
            right = right > x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
        putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    std::string windowName = "3D Objects";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, topviewImg);

    if (bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox,
                              std::vector<cv::KeyPoint> &kptsPrev,
                              std::vector<cv::KeyPoint> &kptsCurr,
                              std::vector<cv::DMatch> &kptMatches)
{
    std::vector<cv::DMatch> matches;
    std::vector<double> matchesDistances;
    double kptMatchesDistancesSum = 0.0;

    for (auto match : kptMatches)
    {
        const cv::KeyPoint kptCurr = kptsCurr[match.trainIdx];
        // use only keypoint matches inside bounding box ROI
        if (boundingBox.roi.contains(kptCurr.pt))
        {
            const cv::KeyPoint kptPrev = kptsPrev[match.queryIdx];
            matches.push_back(match);

            const double distance = cv::norm(kptCurr.pt - kptPrev.pt);
            matchesDistances.push_back(distance);
            kptMatchesDistancesSum += distance;
        }
    }

    const double meanDistance = kptMatchesDistancesSum / matchesDistances.size();
    for (int idx = 0; idx < matchesDistances.size(); ++idx)
    {
        // eliminate outliers
        if (matchesDistances[idx] < meanDistance)
        {
            boundingBox.kptMatches.push_back(matches[idx]);
        }
    }

    // std::cout << "boundingBox.kptMatches.size=" << boundingBox.kptMatches.size() << std::endl;
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev,
                      std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches,
                      double frameRate,
                      double &TTC,
                      cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    std::vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero
                distRatios.push_back(distCurr / distPrev);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    std::sort(distRatios.begin(), distRatios.end());
    const long medIndex = floor(distRatios.size() / 2.0);
    // compute median dist. ratio to remove outlier influence
    double medDistRatio = 0.0;
    if (distRatios.size() % 2 == 0)
    {
        medDistRatio = (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0;
    }
    else
    {
        medDistRatio = distRatios[medIndex];
    }

    TTC = (-1.0 / frameRate) / (1 - medDistRatio);

    // std::cout << "CAMERA TTC: " << TTC << std::endl;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr,
                     double frameRate,
                     double &TTC)
{
    // auxiliary variables
    const double dT = 1.0 / frameRate; // time between two measurements in seconds
    const double laneWidth = 4.0;      // assumed width of the ego lane

    // calculate standard deviation

    double lidarPointsPrevSumX = 0.0; // sum of all previous-frame lidar points' x values
    for (std::vector<LidarPoint>::iterator it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); /*don't increment here*/)
    {
        // remove points outside ego lane
        if (abs(it->y) > (laneWidth / 2.0))
        {
            it = lidarPointsPrev.erase(it);
        }
        else
        {
            lidarPointsPrevSumX += it->x;
            ++it;
        }
    }
    double lidarPointsCurrSumX = 0.0; // sum of all current-frame lidar points' x values
    for (std::vector<LidarPoint>::iterator it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); /*don't increment here*/)
    {
        // remove points outside ego lane
        if (abs(it->y) > (laneWidth / 2.0))
        {
            it = lidarPointsCurr.erase(it);
        }
        else
        {
            lidarPointsCurrSumX += it->x;
            ++it;
        }
    }

    // calculate means of previous and current lidar points' x values
    double lidarPointsPrevMean = lidarPointsPrevSumX / lidarPointsPrev.size();
    double lidarPointsCurrMean = lidarPointsCurrSumX / lidarPointsCurr.size();

    // subtract the mean from each value in the data set(s)
    // square the differences
    // add up the squared differences
    double lidarPointsPrevDeviationsSum = 0.0;
    for (const LidarPoint &p : lidarPointsPrev)
    {
        lidarPointsPrevDeviationsSum += ((p.x - lidarPointsPrevMean) * (p.x - lidarPointsPrevMean));
    }
    double lidarPointsCurrDeviationsSum = 0.0;
    for (const LidarPoint &p : lidarPointsCurr)
    {
        lidarPointsCurrDeviationsSum += ((p.x - lidarPointsCurrMean) * (p.x - lidarPointsCurrMean));
    }

    // divide the total sum(s) from previous step by individual data set size - 1
    // take the square root to get the standard deviation
    double lidarPointsPrevStdDev = sqrt(lidarPointsPrevDeviationsSum / (lidarPointsPrev.size() - 1));
    double lidarPointsCurrStdDev = sqrt(lidarPointsCurrDeviationsSum / (lidarPointsCurr.size() - 1));

    // eliminate outliers and find average distance to lidar points within ego lane
    // use three-sigma rule as threshold
    lidarPointsPrevSumX = 0.0;
    lidarPointsCurrSumX = 0.0;
    uint lidarPointsPrevCountX = 0;
    uint lidarPointsCurrCountX = 0;

    // filter outliers
    for (const auto &p : lidarPointsPrev)
    {
        if (abs(p.x - lidarPointsPrevMean) < (3 * lidarPointsPrevStdDev))
        {
            lidarPointsPrevSumX += p.x;
            ++lidarPointsPrevCountX;
        }
    }
    for (const auto &p : lidarPointsCurr)
    {
        if (abs(p.x - lidarPointsCurrMean) < (3 * lidarPointsCurrStdDev))
        {
            lidarPointsCurrSumX += p.x;
            ++lidarPointsCurrCountX;
        }
    }

    const double d1 = lidarPointsCurrSumX / lidarPointsCurrCountX;
    TTC = (d1 * dT) / ((lidarPointsPrevSumX / lidarPointsPrevCountX) - d1);

    // std::cout << "LIDAR TTC: " << TTC << std::endl;
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches,
                        std::map<int, int> &bbBestMatches,
                        DataFrame &prevFrame,
                        DataFrame &currFrame)
{
    // initialize a null matrix of bounding boxes size prevFrame x currFrame
    int count[prevFrame.boundingBoxes.size()][currFrame.boundingBoxes.size()] = {};

    for (const auto &match : matches)
    {
        // get keypoints with indices from match keypoints
        const cv::KeyPoint prevKeypoint = prevFrame.keypoints[match.queryIdx];
        const cv::KeyPoint currKeypoint = currFrame.keypoints[match.trainIdx];

        // iterate over all bounding boxes in previous frame and count keypoint matches to bounding boxes in current frame
        for (const auto &prevFrameBox : prevFrame.boundingBoxes)
        {
            // check if previous frame keypoint is contained in any bounding box
            if (prevFrameBox.roi.contains(prevKeypoint.pt))
            {
                // check if current frame keypoint is contained in any bounding box
                for (const auto &currFrameBox : currFrame.boundingBoxes)
                {
                    // check if corresponding current frame keypoint is contained in any bounding box
                    if (currFrameBox.roi.contains(currKeypoint.pt))
                    {
                        // two matched keypoints are contained in bounding boxes respectively
                        // mark the bounding box corelation
                        count[prevFrameBox.boxID][currFrameBox.boxID]++;
                    }
                }
            }
        }
    }

    // extract max values
    for (const auto &prevFrameBox : prevFrame.boundingBoxes)
    {
        uint maxCount = 0;
        int maxId = 0;
        for (const auto &currFrameBox : currFrame.boundingBoxes)
        {
            if (count[prevFrameBox.boxID][currFrameBox.boxID] > maxCount)
            {
                maxCount = count[prevFrameBox.boxID][currFrameBox.boxID];
                maxId = currFrameBox.boxID;
            }
        }
        bbBestMatches.insert({prevFrameBox.boxID, maxId});
    }

    // std::cout << "bbBestMatches size: " << bbBestMatches.size() << std::endl;
}
