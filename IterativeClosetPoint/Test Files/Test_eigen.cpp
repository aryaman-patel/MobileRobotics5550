#include<iostream>
#include<eigen3/Eigen/Dense>

//Compile using: g++ -std=c++17 -I/usr/include/eigen3 Test_eigen.cpp -o test_exec -O2 -DNDEBUG
using namespace Eigen;

int main()
{
    int a_n_rows = 4000;
    int a_n_cols = 3000;
    int b_n_rows = a_n_cols;
    int b_n_cols = 200;
    MatrixXi a(a_n_rows,a_n_cols);

    // Filling the a matrix
    for (int i = 0; i < a_n_rows; ++i)
    {
        for (int j =0; j < a_n_cols; ++j)
        {
            a(i,j) = a_n_cols*i + j;
        }
    }

    // Similarly for b
    MatrixXi b(b_n_rows, b_n_cols);
    for (int i = 0; i < a_n_rows; ++i)
    {
        for (int j =0; j < a_n_cols; ++j)
        {
            a(i,j) = a_n_cols*i + j;
        }
    }
    
    MatrixXi d (a_n_rows,b_n_cols);

    clock_t begin = clock();
    d.noalias() = a * b;        // When the left hand term is not involved in the right hand side so we avoid aliasing. 
    clock_t end  = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Time taken : " << elapsed_secs << std::endl;
}