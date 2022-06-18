/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

// Test load pcd
// void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
// {
//     // ----------------------------------------------------
//     // -----Open 3D viewer and display City Block     -----
//     // ----------------------------------------------------

//     // ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
//     ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
//     pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
//     renderPointCloud(viewer, inputCloud, "inputCloud");
// }

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,
               ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // Setting hyper parameters

    // FilterCloud
    const float filterRes = 0.4;
    const Eigen::Vector4f minpoint(-10, -6.5, -2, 1);
    const Eigen::Vector4f maxpoint(30, 6.5, 1, 1);
    // SegmentPlane
    const uint8_t maxIterations = 40;
    const float distanceThreshold = 0.3;
    // Clustering
    const float clusterTolerance = 0.5;
    const uint8_t minsize = 10;
    const uint8_t maxsize = 140;

    // Step 1: Filter cloud to reduce amount of points
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minpoint, maxpoint);

    // Step 2: Segment the filtered cloud into two parts, road and obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
        segmentCloud = pointProcessorI->RansacSegmentPlane(filteredCloud, maxIterations, distanceThreshold);
    // renderPointCloud(viewer, inputCloud, "inputCloud");
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    // Step 3: Cluster the obstacle cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
        cloudClusters = pointProcessorI->EuclideanClustering(segmentCloud.first, clusterTolerance, minsize, maxsize);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size: ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);

        // Step 4: Bounding boxes for each obstacle cluster
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // cityBlock(viewer);

    //  Stream cityBlock function
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
        {
            streamIterator = stream.begin();
        }

        viewer->spinOnce();
    }
}
