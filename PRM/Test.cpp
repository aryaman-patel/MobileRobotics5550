#include <eigen3/Eigen/Core>
#include <random>
#include <cstdlib>
#include <ctime>
#include <iostream>

int main() {
    // Create a 4x2 matrix
    Eigen::MatrixXi points(4,2);
    points << 1, 2,
              3, 4,
              5, 6,
              7, 8;
    
    // Print the original matrix
    std::cout << "Original matrix:" << std::endl;
    std::cout << points << std::endl;
    
    // Reverse the rows of the matrix
    std::cout << "Reversed matrix:" << std::endl;
    std::cout << points.colwise().reverse() << std::endl;
    
   
    return 0;
}