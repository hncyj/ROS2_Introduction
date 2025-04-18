#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

int main() {
    // 创建一个点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 生成一些随机点
    cloud->width = 10;
    cloud->height = 1; // 1D point cloud
    cloud->points.resize(cloud->width * cloud->height);
    
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
        cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
        cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
    }

    std::cout << "Generated " << cloud->points.size() << " random points." << std::endl;

    // 创建 KD-Tree 结构
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 选择一个查询点
    pcl::PointXYZ searchPoint = cloud->points[0];  // 选第一个点作为查询点
    std::cout << "Search Point: (" << searchPoint.x << ", " << searchPoint.y << ", " << searchPoint.z << ")" << std::endl;

    /*** 方式 1：K 近邻搜索 (K-Nearest Neighbors) ***/
    int K = 3;  // 查找 3 个最近邻
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    std::cout << "\nK nearest neighbor search (K=" << K << "):" << std::endl;
    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
            std::cout << " " << cloud->points[pointIdxNKNSearch[i]].x
                      << " " << cloud->points[pointIdxNKNSearch[i]].y
                      << " " << cloud->points[pointIdxNKNSearch[i]].z
                      << " (distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        }
    }

    /*** 方式 2：半径搜索 (Radius Search) ***/
    float radius = 100.0;  // 设定搜索半径
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    std::cout << "\nRadius search (radius=" << radius << "):" << std::endl;
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            std::cout << " " << cloud->points[pointIdxRadiusSearch[i]].x
                      << " " << cloud->points[pointIdxRadiusSearch[i]].y
                      << " " << cloud->points[pointIdxRadiusSearch[i]].z
                      << " (distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
        }
    }

    return 0;
}