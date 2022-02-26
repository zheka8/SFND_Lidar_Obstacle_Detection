/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display city block from pcd file  -----
    // ----------------------------------------------------

    // perform voxel filtering
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
    filterCloud = pointProcessor->FilterCloud(inputCloud, 0.2f , Eigen::Vector4f (-20, -6, -40, 1.), Eigen::Vector4f (20., 6., 40., 1.));
    
    // segment into road and objects
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(filterCloud, 100, 0.3);
    //renderPointCloud(viewer, segmentCloud.first, "cloudObst", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "cloudRoad", Color(0,1,0));


    // further divide objects into clusters
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 0.8, 10, 1000);
    std::vector<std::vector<int>> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 0.4, 10, 1000);

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,0,1), Color(0,1,1), Color(1,0,1), Color(1,1,0)};
  	for(std::vector<int> cluster : cloudClusters)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
  		for(int indice: cluster)
  			//clusterCloud->points.push_back(pcl::PointXYZI(points[indice][0],points[indice][1],points[indice][2]));
              clusterCloud->points.push_back(segmentCloud.first->points[indice]);
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%colors.size()]);
  		++clusterId;
  	}

    /*
    // render clusters
    int clusterId = 0;
    std::vector<Color> colors = {Color(0,1,1), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        //renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%(colors.size())]);
        
        // add bounding boxes
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        
        ++clusterId;
    }
    */

    /*
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%(colors.size())]);
        
        // add bounding boxes
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        
        ++clusterId;
    }
    */

    //renderPointCloud(viewer, filterCloud, "inputCloud");
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    bool motion = false;

    if(motion)
    {
        //setup processor for STREAM
        ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
        std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1/");
        auto streamIterator = stream.begin();
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

        while (!viewer->wasStopped ())
        {
            // Clear viewer
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();

            // Load pcd and run obstacle detection process
            inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
            cityBlock(viewer, pointProcessorI, inputCloudI);

            streamIterator++;
            if(streamIterator == stream.end())
                streamIterator = stream.begin();

            viewer->spinOnce ();
        }
    }
    else
    {
        //setup processor for NON-STREAM
        ProcessPointClouds<pcl::PointXYZI>* pointProcessorNS = new ProcessPointClouds<pcl::PointXYZI>();
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI = pointProcessorNS->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
        cityBlock(viewer, pointProcessorNS, inputCloudI);

        // display NON-STREAM
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce ();
        } 
    }

}