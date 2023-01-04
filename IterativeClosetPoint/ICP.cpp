#include<iostream>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/SVD>
#include<fstream>
#include<vector>

// Run using: g++ -std=c++17 -I/usr/include/eigen3 ICP.cpp -o ICP -O2 -DNDEBUG

using namespace Eigen;

struct Tyx
{
    MatrixXd Rot;
    VectorXd trans;
    MatrixXi Corresp;
};

// Rigid Registration.
Tyx ComputeOptimalRigidRegistration(MatrixXd X, MatrixXd Y, MatrixXi C)
{
    Tyx T;
    // Calculate the point cloud centroids.
    MatrixXd x_subset = X(C.col(0), Eigen::placeholders::all);
    MatrixXd y_subset = Y(C.col(1), Eigen::placeholders::all);
    VectorXd x_centroid = x_subset.colwise().mean();
    VectorXd y_centroid = y_subset.colwise().mean();

    // Calculate the deviation of X and Y.
    MatrixXd x_deviation = x_subset.rowwise() - x_centroid.transpose();
    MatrixXd y_deviation = y_subset.rowwise() - y_centroid.transpose();

    // Calculate covariance.
    MatrixXd W = x_deviation.transpose() * y_deviation;
    //std::cout << "W covarieance matrix: " << W << std::endl;

    //Calculate the SVD.
    JacobiSVD<MatrixXd> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    MatrixXd U = svd.matrixU();
    MatrixXd V = svd.matrixV();
    //std::cout << "U = " << U << std::endl;
    //std::cout << "V = " << V << std::endl;

    // Construct optimal rotation
    T.Rot = U * V.transpose();
    //std::cout << "T.Rot = " << T.Rot << std::endl;

    // Construct Optimal translation
    T.trans = y_centroid - (x_centroid.transpose()*T.Rot).transpose();
    //std::cout << "T.trans = " << T.trans << std::endl;

    return T;
}


// Get the correspondences. 
MatrixXi EstimateCorrespondences(MatrixXd X, MatrixXd Y, Vector3d t, Matrix3d R, double d_max)
{
    //Initialize C as empty.
    MatrixXi C(0,2);
    int N = X.rows();
    MatrixXd transposed_x(N,3);
    int index;
    VectorXd norm(N);
    for(int i = 0; i < X.rows(); i++)
    {
        transposed_x.row(i) = X.row(i) * R + t.transpose();
    }
    // Find the indexes in the least square sense. 
    for(int i =0; i < transposed_x.rows(); i++)
    {
        // diff = Y.rowwise() - transposed_x.row(i);
        // norm = diff.rowwise().norm();
        norm = (Y.rowwise() - transposed_x.row(i)).rowwise().norm();
        // Print the first 10 elements of the norm vector. Debugging purposes.
        // if (i == 0)
        // {
        //     std::cout << "X = " << X.row(0) << std::endl;
        //     std::cout << "Y = " << Y.row(0) << std::endl;
        //     std::cout << "transposed_x = " << transposed_x.row(0) << std::endl;
        //     std::cout << norm.head(10) << std::endl;            
        // }
        norm.minCoeff(&index);
        if (norm.coeff(index) < d_max)
        {
            C.conservativeResize(C.rows()+1, Eigen::NoChange);
            C.row(C.rows()-1) << i,index;
        }
    }
    return C;
}


Tyx ICP_algo(MatrixXd X, MatrixXd Y, Vector3d t0, Matrix3d R0, double d_max, int max_iter)
{
    int iter = 0;
    Tyx T;
    MatrixXi C;
    while(true)
    {
        C = EstimateCorrespondences(X , Y , t0 , R0 , d_max);
        T = ComputeOptimalRigidRegistration(X, Y, C);
        t0 = T.trans;
        R0 = T.Rot;
        iter = iter +1;
        if (iter == max_iter)
        {
            std::cout << "Max iterations reached " << iter << std::endl;
            break;
        }
    }
    T.Corresp = C;

    return T;
}



int main()
{

    // Create a vector of vector
    std::vector<std::vector<double>> data;
    double value;
    //Open the text file.
    std::ifstream file("pclX.txt");
    //Read the data from the file to the matrix
    while(file >> value)
    {
        data.push_back({value});
        while (file.peek() == ' ')
        {
            file >> value;
            data.back().push_back(value);
        }
    }
    file.close();
    int n_rows = data.size();
    int n_cols = data[0].size();
    MatrixXd pcl_X(n_rows,n_cols);

    for (int i = 0; i < n_rows; ++i)
    {
        for (int j =0; j < n_cols; ++j)
        {
            pcl_X(i,j) = data[i][j];
        }
    }
    file.close();
    // Similarly for pcl_Y
    std::ifstream file_y("pclY.txt");
    //Read the data from the file to the matrix
    // Clear the data vector
    data.clear();
    while(file_y >> value)
    {
        data.push_back({value});
        while (file_y.peek() == ' ')
        {
            file_y >> value;
            data.back().push_back(value);
        }
    }
    file_y.close();
    n_rows = data.size();
    n_cols = data[0].size();
    MatrixXd pcl_Y(n_rows,n_cols);

    for (int i = 0; i < n_rows; i++)
    {
        for (int j =0; j < n_cols; j++)
        {
            pcl_Y(i,j) = data[i][j];
        }
    }

    // Initial translational vector and Rotational matrix
    Vector3d t = Vector3d::Zero();
    Matrix3d R = Matrix3d::Identity();  // Create a identity matrix.
    double d_max = 0.25;
    int iter = 30;
    Tyx result;

    // std::cout << "pcl_X = " << pcl_X.row(0) << std::endl;
    // std::cout << "pcl_Y = " << pcl_Y.row(0) << std::endl;
    clock_t begin = clock();
    result = ICP_algo(pcl_X, pcl_Y, t, R, d_max, iter);
    clock_t end  = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Time taken : " << elapsed_secs << std::endl;

    std::cout << "\n Translation vector: " << result.trans << std::endl;
    std::cout << "-----------------------------" << std::endl;
    std::cout << "Rotation matrix: " << result.Rot << std::endl;

    MatrixXd result_pt_cloud(pcl_X.rows(),3);
    for(int i = 0; i < pcl_X.rows(); i++)
    {
        result_pt_cloud.row(i) = pcl_X.row(i) * result.Rot + result.trans.transpose();
    }
    // Write the result point cloud to a text file.
    std::ofstream file_result("result.txt");
    for(int i = 0; i < result_pt_cloud.rows(); i++)
    {
        file_result << result_pt_cloud(i,0) << " " << result_pt_cloud(i,1) << " " << result_pt_cloud(i,2) << std::endl;
    }

    return 0;

}