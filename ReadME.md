# MobileRobotics5550
This is a repository containing my implementation of few of the algorithms I have written as a part of my EECE5550 Mobile Robotics course. 

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

## Route planning in occupancy grid maps -
The following figure shows a occupancy grid map, which is a convenient way to represent information of the robot's environment, and well suited to route planning algorithms.
In this exercise, I implemented two graph-based planning algorithms, A* search and Probablistic Roadmap to perform route planning. ![occupancy_map](https://user-images.githubusercontent.com/117113574/211173947-75cc7245-a583-4129-863b-bfa58e30bc05.png)
### A* Search output -
A* is a popular path planning algorithm that works by using a heuristic function to guide the search for a path through the environment. A* expands nodes in the search tree based on an estimate of the minimum total cost from the starting point to the goal through that node. This estimate, known as the "heuristic," guides the search towards the goal and allows A* to find optimal paths more quickly than other search algorithms. However, A* requires a complete map of the environment in advance and can be computationally expensive, as it may need to search a large portion of the space to find a solution.

![A_star_optimal_path](https://user-images.githubusercontent.com/117113574/211173995-61bcacfe-c8ec-4734-a17c-01812a350c1a.png)

### PRM's output - 
PRM is a sampling-based path planning algorithm that works by constructing a roadmap of the free space in the environment and then using this roadmap to find a path between the starting point and the goal location. One of the main advantages of PRM is that it can handle environments with complex or unknown geometry, as it does not require a complete map of the environment in advance. It can also find paths in real-time, as it does not need to search the entire space to find a solution. However, PRM can sometimes produce suboptimal paths, as it relies on a random sampling of the space and does not take into account the specific characteristics of the environment.

The image below is the all the connected nodes of the sampled points based on the rechability check performed. 

![PRM_Nodes](https://user-images.githubusercontent.com/117113574/211174058-eca6bc34-3c21-47d1-93d1-087e649228a8.png)

The final output with sampled points-

![prm](https://user-images.githubusercontent.com/117113574/211174071-dc3a9822-6206-4694-b0da-1034e5425b76.png)


## State estimation by (Monte-Carlo) Partile Filter on a Lie Group -
In this exercise, I have applied particle filtering to perform state estimation over a Lie group:
specifically, designed and implement a particle filter to track the pose of a differential-drive
ground robot.

We can write a generative description of the motion model for a a differential drive robot. Suppose the left and right wheel, with $r$ as the wheel radius, $w$ as the track width, true speeds defined as $(\tilde{\varphi}_l, \tilde{\varphi}_r)$ with $commanded$ wheel speeds of $u := (\dot{\varphi}_l,\dot{\varphi}_r)$ has a noise model of:

$\tilde{\varphi}_l = \dot{\varphi}_l + \epsilon_l,\ \ \epsilon_l \sim N(0,\sigma_l^2)$
 
$\tilde{\varphi}_r = \dot{\varphi}_r + \epsilon_r,\ \ \epsilon_r \sim N(0,\sigma_r^2)$

The generative motion model $p(x_{t2} | x_{t1} , \dot{\varphi_l},\dot{\varphi_r}, r, w, \sigma_l, \sigma_r)$ that parameterizes the distribution of the pose of the robot $x_{t2} \in SE(2)$ as a function of the pose $x_{t1} \in SE(2)$ at time $t_2$ is given by the exponential map:

$P(X_0 | X_t, u) = P(X_0)*exp(t * \dot{\Omega}(\dot{\varphi}_l,\dot{\varphi}_r))$

where $\dot{\Omega}$ is an element in the $Lie(SE(2))$ characterized by the wheel speeds $(\dot{\varphi}_l,\dot{\varphi}_r)$

Output by Particle Filter for the time stamps [0,5,10,15,20] with the measurement updates -

![output](https://user-images.githubusercontent.com/117113574/211174996-e9fd8539-6936-4ea7-873e-179e67bd53c9.png)


