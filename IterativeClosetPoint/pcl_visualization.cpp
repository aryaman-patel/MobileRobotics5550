#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<fstream>
#include<vector>

// Info: 
// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>)
// pcl::PointCloud is a template class that for storing point cloud of the type pcl::PointXYZ which is a point cloud of 
// that has 3 fields 'x', 'y' , 'z' that stores the 3D coordinates of the points. 
// the Ptr is a smart pointer similar to the std::shared_ptr in C++.



Eigen::MatrixXd Txt_to_eigen (std::ifstream &file)
{
    std::vector<std::vector<double>> data;
    double value;
    if (file.fail())
    {
        std::cerr << "Error opening input file" << std::endl;
        return Eigen::MatrixXd();
    }
    // Read the data from the file to the matrix
    while (file >> value)
    {
        data.push_back({value});
        while (file.peek() == ' ')
        {
            file >> value;
            data.back().push_back(value);
        }
    }
    std::cout << data.size() << std::endl;
    int n_rows = data.size();
    int n_cols = data[0].size();
    Eigen::MatrixXd pointCloud(n_rows, n_cols);
    for (int i = 0; i < n_rows; ++i)
    {
        for (int j = 0; j < n_cols; ++j)
        {
            pointCloud(i, j) = data[i][j];
        }
    }
    data.clear();
    file.close();
    return pointCloud;
}


void populatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::MatrixXd& data)
{
    int n_rows = data.rows();
    for (int i = 0; i < n_rows; ++i)
    {
        pcl::PointXYZ point;
        point.x = data(i, 0);
        point.y = data(i, 1);
        point.z = data(i, 2);
        cloud->points.push_back(point);
    }
}


int main()
{

    // Open the text file.
    std::ifstream file("result.txt");
    std::ifstream file2("pclX.txt");
    std::ifstream file3("pclY.txt");
    // Read the data from the file to the matrix

    Eigen::MatrixXd PC_result = Txt_to_eigen(file);
    Eigen::MatrixXd PC_X = Txt_to_eigen(file2);
    Eigen::MatrixXd PC_Y = Txt_to_eigen(file3);
    // Create a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);

    populatePointCloud(cloud, PC_result);
    populatePointCloud(cloud2, PC_X);
    populatePointCloud(cloud3, PC_Y);

   
    // Display the point cloud
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    // viewer.showCloud(cloud);
    viewer.showCloud(cloud);
    viewer.showCloud(cloud2,"cloud2");
    viewer.showCloud(cloud3,"cloud3");

    while (!viewer.wasStopped())
    {
    }
}
