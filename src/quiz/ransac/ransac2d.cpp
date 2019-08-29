#include <random>

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	std::cout << "Entering... rand max = " << RAND_MAX << std::endl;

    while (maxIterations--) {
		std::unordered_set<int> inliers;
        while (inliers.size() < 3) {
            int idx = rand() % cloud->points.size();
            inliers.insert(idx);
        }

        auto itr = inliers.begin();
		float x1 = cloud->points[*itr].x; // pt A
		float y1 = cloud->points[*itr].y;
        float z1 = cloud->points[*itr].z;
        itr++;
        float x2 = cloud->points[*itr].x; // pt B
        float y2 = cloud->points[*itr].y;
        float z2 = cloud->points[*itr].z;
        itr++;
        float x3 = cloud->points[*itr].x; // pt C
        float y3 = cloud->points[*itr].y;
        float z3 = cloud->points[*itr].z;

        float A_C_x = x1 - x3; // U
        float A_C_y = y1 - y3;
        float A_C_z = z1 - z3;

        float A_B_x = x1 - x2; // V
        float A_B_y = y1 - y2;
        float A_B_z = z1 - z2;

        float A = A_C_y*A_B_z - A_B_y*A_C_z;
        float B = A_B_x*A_C_z - A_C_x*A_B_z;
        float C = A_C_x*A_B_y - A_B_x*A_C_y;

        float D = A*x1 + B*y1 + D*z1;

		for(int j = 0; j < cloud->size(); j++) {
			pcl::PointXYZ _pt = cloud->points[j];
			float _x1 = _pt.x;
			float _y1 = _pt.y;
            float _z1 = _pt.z;
			float dist = abs(A*_x1 + B*_y1 + C*_z1 + D) / sqrt(A*A + B*B + C*C);
			if (dist < distanceTol) {
				inliers.insert(j);
			}
		}

		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}

	}

	std::cout << "Inliers size is: " << inliersResult.size() << std::endl;
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 0.4);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}

  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}

}
