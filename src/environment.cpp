#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene) {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    Lidar* lidar = new Lidar(cars, 0.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    ProcessPointClouds<pcl::PointXYZ> pointProcessor = ProcessPointClouds<pcl::PointXYZ>();
    auto segmentCloud = pointProcessor.SegmentPlane(cloud, 100, 0.2);
    auto road_pts = segmentCloud.first;
    auto car_pts = segmentCloud.second;

    // renderPointCloud(viewer, car_pts, "cars", Color(1,0,0));
    renderPointCloud(viewer, road_pts, "road", Color(1,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.second, 1.0, 3, 30);
    int cluster_ID = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        std::cout << "cluster size ";
        renderPointCloud(viewer, cluster, "ObstCloud" + std::to_string(cluster_ID), colors[cluster_ID]);
        cluster_ID++;

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, cluster_ID);
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
  // ----------------------------------------------------
  // ------- Open 3D viewer and display City Block ------
  // ----------------------------------------------------

  auto filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.25f,
      Eigen::Vector4f(-20, -4, -20, 1), Eigen::Vector4f (30, 4, 20, 1));

  auto segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
  auto road_pts = segmentCloud.first;
  auto car_pts = segmentCloud.second;

  renderPointCloud(viewer, road_pts, "road_pts", Color(0,1,0));
  // renderPointCloud(viewer, car_pts, "car_pts", Color(1,0,0));
                                                                                 // clusterTolerance, minSize, maxSize
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(car_pts, 1.0, 30, 3000);
  int cluster_ID = 0;
  std::cout << "Found " << cloudClusters.size() << " clusters!" << std::endl;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
      std::cout << "cluster ID " << cluster_ID << std::endl;
      renderPointCloud(viewer, cluster, "ObstCloud" + std::to_string(cluster_ID), colors[cluster_ID%4]);
      cluster_ID++;

      Box box = pointProcessorI->BoundingBox(cluster);
      renderBox(viewer, box, cluster_ID);
  }
}


// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
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

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped()) {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }

        viewer->spinOnce();
  }

}

