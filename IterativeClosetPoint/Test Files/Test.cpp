#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>




int main()
{
	Eigen::MatrixXd B(3, 3);
	B << 1, 2, 3,
		4, 5, 6,
		7, 8, 9;

	Eigen::VectorXd u(3);
	u << 3, 4, 5;

	Eigen::MatrixXd C = B.rowwise() - u.transpose();


	std::cout << C << std::endl;


}