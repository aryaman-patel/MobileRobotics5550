# MobileRobotics5550
This is a repository contraining my implementation of few of the algorithms I have written as a part of my EECE5550 Mobile Robotics course. 

## Scan matching using Iterative Closest Point
In this I have implement the Iterative Closest Point (ICP) algorithm, and use it
to estimate the rigid transformation that optimally aligns two 3D pointclouds. The given
two pointclouds $X,Y \subset \mathbb{R}^{d}$ have an initial guess $(t_0,R_0) \in SE(d)$ for the optimal rigid registration $y = R_x + t$ aligning $X$ to $Y$. 

The python and C++ implementations have been included in the `IterativeClosestPoint` folder. 
### Instructions to run the C++ implementation: 
Make sure your system has the latest library for [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) and [PCL](https://pointclouds.org/) installed. 
To run the source file `ICP.cpp` use the following commands:
```
g++ -std=c++17 -I/usr/include/eigen3 ICP.cpp -o ICP -O2 -DNDEBUG
```
Here, the flags: `-O2` used in the compiler to optimize the genrated code for the maximum performance. Thie flag enables a number of optimization options that can improve the speed and efficiency. The flag `-DNDEBUG` tells the compiler to disable debugging information. This improves the performance of the code by eliminating the checks and other overheads that are not necessary to build. 
 
The `ICP.cpp` code creates a resultant file called `result.txt` that stores the resultant points of the pointcloud.

In order, to visualize the point cloud I have written an implementation using the [PCL](https://pointclouds.org/) library. Run the following commands:
For the code to run make sure to move a copy of `pclX.txt`, `pclY.txt` and `result.txt` file to the `build` folder.

```
cd build
cmake ..
make
./point_cloud_visualization
```
The resultant point cloud in comparision to the original ones looks like this:
![PCL](https://user-images.githubusercontent.com/117113574/210670258-9c4e113f-fc7f-473a-b349-026e137d9d5f.png)
