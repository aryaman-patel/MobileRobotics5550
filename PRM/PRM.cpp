#include<iostream>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Core>
#include<opencv2/opencv.hpp>
#include <random>
#include <ctime>
#include <cstdlib>


// Rejection sampling.
Eigen::Vector2i sampling(Eigen::MatrixXd& occupancy_grid)
{
    while (true)
    {

        int r = std::rand() % occupancy_grid.rows();
        int c = std::rand() % occupancy_grid.cols();
        if (occupancy_grid[r,c] == 1)
        {
            Eigen::Vector2i vertex(r,c);
            break;
        }
    }
}


// Get all the points on the line between the two points. 
Eigen::MatrixXi get_line(Eigen::Vector2i p1, Eigen::Vector2i p2)
{
    Eigen::MatrixXi points(0,2);
    bool isstep = std::abs(p2[1] - p1[1]) > std::abs(p2[0] - p1[0]);
    if (isstep)
    {
        p1[0], p1[1] = p1[1] , p1[0];
        p2[0], p2[1] = p2[1] , p2[0];
    }
    bool rev = false;
    if (p1[0] > p2[0])
    {
        p1[0], p2[0] = p2[0] , p1[0];
        p1[1], p2[1] = p2[1] , p1[1];
        rev = true;
    }
    int deltax = p2[0] - p1[0];
    int deltay = std::abs(p2[1] - p1[2]);
    int error = deltax/2;
    int y = p1[1];
    int ystep = NULL;
    if (p1[1] < p2[1])
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }
    for(int x = p1[0]; x <= p2[0]; x++)
    {
        if (isstep)
        {
            points.conservativeResize(points.rows()+1, Eigen::NoChange);
            points.row(points.rows()-1) << y,x;
        }
        else
        {
            points.conservativeResize(points.rows()+1, Eigen::NoChange);
            points.row(points.rows()-1) << x,y;
        }
        error -= deltay;
        if (error < 0)
        {
            y += ystep;
            error += deltax;
        }
    }
    if (rev)
    {
        points.colwise().reverse();
    }

    return points;
}



bool reachability_check(Eigen::Vector2i p1, Eigen::Vector2i p2)
{
    Eigen::MatrixXi points
}

int main(int argc, char** argv)
{
    // Seed to generate the random numbers. 
    std::srand(std::time(NULL));
    // Get the occupancy grid from the image
    cv::Mat occupancy_map = cv::imread("occupancy_map.png", CV_LOAD_IMAGE_GRAYSCALE);

    // Convert the image to the biary array and Eigen Matrix
    cv::Mat occupancy_grid;
    cv::threshold(occupancy_map, occupancy_grid,128,1, CV_THRESH_BINARY);

    Eigen::MatrixXd occupancy_grid = cv::cv2eigen(occupancy_grid);

    // Start and End points:
    Eigen::Vector2i start(635,140);
    Eigen::Vector2i end(350,400);
    int sampling = 2500;
    int d_max = 75;
    
}

