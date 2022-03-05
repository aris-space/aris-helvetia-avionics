#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#ifndef MATH_UTILS_H_
#define MATH_UTILS_H_

// Calculates the transpose of a Matrix
void eye(int n, float A[n][n]);

// Calculates the transpose of a Matrix
void transpose(int m, int n, float A[m][n], float A_T[n][m]);

// Add vector b to a. All vectors have size n
void vecadd(int n, float a[n], float b[n], float c[n]);

// Substract vector b from a
void vecsub(int n, float a[n], float b[n], float c[n]);

// Multiply a scalar with a vector of size n
void scalarvecprod(int n, float scalar, float a[n], float b[n]);

// Add two matrices A and B with size mxn
void matadd(int m, int n, float A[m][n], float B[m][n], float C[m][n]);

// Substracting matrix B of A both with size mxn
void matsub(int m, int n, float A[m][n], float B[m][n], float C[m][n]);

// Multiplication of two matrices A and B
void matmul(int m, int n, int o, float A[m][n], float B[n][o], float C[m][o], bool reset);

// Multiplicate matrix A with a vector b
void matvecprod(int m, int n, float A[m][n], float b[n], float c[m], bool reset);

// Multiplicate a matrix A by a scalar
void scalarmatprod(int m, int n, float scalar, float A[m][n], float B[m][n]);

// Create the sum of a vector a
float vecsum(int n, float a[n]);

// Vector product of two 3dim vectors
void veccrossprod(float a[3], float b[3], float c[3]);

// Find maximum between two numbers.
int max(int num1, int num2);

// Find minimum between two numbers.
int min(int num1, int num2);

// Function to get cofactor of A[p][q] in temp[][]. n is current dimension of A[][]
void cofactor(int dim, float A[dim][dim], float temp[dim][dim], int p, int q, int n);

// Recursive function for finding determinant of matrix. n is current dimension of A[][].
float determinant(int dim, float A[dim][dim], int n);

// Function to get adjoint of A[dim][dim] in adj[dim][dim].
void adjoint(int dim, float A[dim][dim], float adj[dim][dim]);

// Function to calculate and store inverse, returns false if matrix is singular
bool inverse(int dim, float A[dim][dim], float A_inv[dim][dim], float lambda);

// Damped Moore-Penrose pseudo-inverse - ETHZ Robot Dynamics Lecture notes
bool pseudo_inverse(int m, int n, float A[m][n], float A_inv[n][m], float lambda);

// the inverse of a matrix which has only diagonal elements
void diag_inverse(int n, float A[n][n], float inverse[n][n], float lambda);

// computes in-place the inverse of the lower triangular matrix L
int lower_triangular_inverse(int n, float *L);

// help function for the cholesky inverse
void cholesky(int n, float A[n][n], float L[n][n]);

// computes the inverse of a Hermitian, positive-definite matrix of dimension n x n using cholesky decomposition
void cholesky_inverse(int n, float A[n][n], float inverse[n][n], float lambda);

// Linear interpolation of two data points
void interpolate(float y[2], float x[2], float xp, float *yp);

// Calculates a polynome which fits the given data
int polyfit(const float* const dependentValues, const float* const independentValues, unsigned int countOfElements, unsigned int order, double* coefficients);

// Discretize state space matrices A and B by using first and second order taylor approximations into Ad and Bd
void discretize(float frequency, int n, int m, float A[n][n], float B[n][m], float Ad[n][n], float Bd[n][m]);

// Normalize quaternion a+bi+cj+dk such that a^2+b^2+c^2+d^2 = 1
void normalize_quarternion(float Q[4]);

// Calculates the rotation matrix 3-2-1 from a quaternion [q1,q2,q3,q0] representation (see Matlab code for sources of eqn)
void rot_from_quat(float Q[4], float rotation_matrix[3][3]);

// Calculates rotation matrix 3-2-1 from euler angles [phi,theta,psi]
void rot_from_euler(float angles[3], float R[3][3]);

// Converts a quaternion [q1,q2,q3,q0] to euler angles [phi,theta,psi]
void euler_from_quat(float Q[4], float angles[3]);

// Converts euler angles [phi,theta,psi] to quaternion [q1,q2,q3,q0] for all calculations rotation matrix is 3-2-1
void quat_from_euler(float angles[3],float Q[4]);

// Calculates the multiplication of two quaternions Q1 and Q2
void quat_prod(float Q1[4],float Q2[4],float Q3[4]);

// Creates a skew matrix from a given vector
void skew(float vector[3], float skew_matrix[3][3]);

// Creates Xi matrix from quaternion [q1,q2,q3,q0]
void Xi_from_quat(float Q[4], float Xi[4][3]);

float euclidean_norm(int n, const float a[n]);

#endif
