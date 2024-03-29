// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> cropBox(true);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(cloudFiltered);
    cropBox.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(const std::unordered_set<int> inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloudObst { new pcl::PointCloud<PointT> ()};
    typename pcl::PointCloud<PointT>::Ptr cloudRoad { new pcl::PointCloud<PointT> ()};
    
	// Assign points to separate clouds
	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliers.count(index))
			cloudRoad->points.push_back(point);
		else
			cloudObst->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudObst, cloudRoad);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Perform RANSAC
    std::unordered_set<int> inliersResult;
    float x1, x2, x3, y1, y2, y3, z1, z2, z3;
    float a, b, c, d;
    float denom, dist;

	while(maxIterations--)
	{
		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));

		// Get the coordinates of selected points
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;

		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;

		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		// Calculate line equation constants
		a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		d = -(a*x1+b*y1+c*z1);

        denom = sqrt(a*a + b*b + c*c);

		// Measure distance between every point and fitted line
		for(int idx = 0; idx < cloud->points.size(); idx++)
		{
			// Don't use points that are already part of the line
			if(inliers.count(idx) > 0)
				continue;

			// Calculate distance of point to line
			PointT point = cloud->points[idx];
			dist = fabs(a*point.x + b*point.y + c*point.z + d)/denom;

			// If distance is smaller than threshold count it as inlier
			if (dist < distanceThreshold)
				inliers.insert(idx);

		}

		// Keep track of max inliers
		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult,cloud);
    return segResult;
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

	// create a KDtree and populate it
	KdTree* tree = new KdTree;
	for(int idx = 0; idx < cloud->points.size(); idx++)
	{
		std::vector<float> p {cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z};
		tree->insert(p, idx);
	}

	// perform clustering
	std::vector<std::vector<int>> clusters = euclideanCluster(tree, clusterTolerance, minSize, maxSize, cloud);

	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(KdTree* tree, float distanceTol, int minSize, int maxSize, typename pcl::PointCloud<PointT>::Ptr cloud)
{

	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(cloud->points.size(), false); 

    //Iterate through each point
	for(int i = 0; i < cloud->points.size(); i++)
	{
        //If point has not been processed
		if(!processed[i])
		{
            //Create cluster, which contains point ids
			std::vector<int> cluster;

            //add all nearby points to this cluster
			proximity(tree, distanceTol, i, processed, cluster, cloud);

            //add clusters
			if(cluster.size() >= minSize && cluster.size() <= maxSize)
			{
				clusters.push_back(cluster);
			}
		}
	}
     
	return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(KdTree* tree, float distanceTol, int pointID, std::vector<bool>& processed, std::vector<int>& cluster, typename pcl::PointCloud<PointT>::Ptr cloud)
{
	processed[pointID] = true;
	cluster.push_back(pointID);
	std::vector<float> p {cloud->points[pointID].x, cloud->points[pointID].y, cloud->points[pointID].z};
	std::vector<int> neighbors = tree->search(p, distanceTol);
	for(int i : neighbors)
	{
		if(!processed[i])
			proximity(tree, distanceTol, i, processed, cluster, cloud);
		else
			cluster.push_back(i);
	}
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}