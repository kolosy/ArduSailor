////////////////////////////////////////////////////////////////////////////////
// File: choleski.c                                                           //
// Contents:                                                                  //
//    Choleski_LU_Decomposition                                               //
//    Choleski_LU_Solve                                                       //
//    Choleski_LU_Inverse                                                     //
//                                                                            //
// Required Externally Defined Routines:                                      //
//    Lower_Triangular_Solve                                                  //
//    Lower_Triangular_Inverse                                                //
//    Upper_Triangular_Solve                                                  //
////////////////////////////////////////////////////////////////////////////////

#include <math.h>                                       // required for sqrt()

//                    Required Externally Defined Routines 
int Lower_Triangular_Solve(double *L, double B[], double x[], int n);
int Lower_Triangular_Inverse(double *L, int n);
int Upper_Triangular_Solve(double *U, double B[], double x[], int n);

////////////////////////////////////////////////////////////////////////////////
//  int Choleski_LU_Decomposition(double *A, int n)                           //
//                                                                            //
//  Description:                                                              //
//     This routine uses Choleski's method to decompose the n x n positive    //
//     definite symmetric matrix A into the product of a lower triangular     //
//     matrix L and an upper triangular matrix U equal to the transpose of L. //
//     The original matrix A is replaced by L and U with L stored in the      //
//     lower triangular part of A and the transpose U in the upper triangular //
//     part of A. The original matrix A is therefore destroyed.               //
//                                                                            //
//     Choleski's decomposition is performed by evaluating, in order, the     //
//     following pair of expressions for k = 0, ... ,n-1 :                    //
//       L[k][k] = sqrt( A[k][k] - ( L[k][0] ^ 2 + ... + L[k][k-1] ^ 2 ) )    //
//       L[i][k] = (A[i][k] - (L[i][0]*L[k][0] + ... + L[i][k-1]*L[k][k-1]))  //
//                          / L[k][k]                                         //
//     and subsequently setting                                               //
//       U[k][i] = L[i][k], for i = k+1, ... , n-1.                           //
//                                                                            //
//     After performing the LU decomposition for A, call Choleski_LU_Solve    //
//     to solve the equation Ax = B or call Choleski_LU_Inverse to calculate  //
//     the inverse of A.                                                      //
//                                                                            //
//  Arguments:                                                                //
//     double *A   On input, the pointer to the first element of the matrix   //
//                 A[n][n].  On output, the matrix A is replaced by the lower //
//                 and upper triangular Choleski factorizations of A.         //
//     int     n   The number of rows and/or columns of the matrix A.         //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix A is not positive definite symmetric (within   //
//                  working accuracy).                                        //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N];                                                        //
//                                                                            //
//     (your code to initialize the matrix A)                                 //
//     err = Choleski_LU_Decomposition((double *) A, N);                      //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else { printf(" The LLt decomposition of A is \n");                    //
//           ...                                                              //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Choleski_LU_Decomposition(double *A, int n)
{
   int i, k, p;
   double *p_Lk0;                   // pointer to L[k][0]
   double *p_Lkp;                   // pointer to L[k][p]  
   double *p_Lkk;                   // pointer to diagonal element on row k.
   double *p_Li0;                   // pointer to L[i][0]
   double reciprocal;

   for (k = 0, p_Lk0 = A; k < n; p_Lk0 += n, k++) {
           
//            Update pointer to row k diagonal element.   

      p_Lkk = p_Lk0 + k;

//            Calculate the difference of the diagonal element in row k
//            from the sum of squares of elements row k from column 0 to 
//            column k-1.

      for (p = 0, p_Lkp = p_Lk0; p < k; p_Lkp += 1,  p++)
         *p_Lkk -= *p_Lkp * *p_Lkp;

//            If diagonal element is not positive, return the error code,
//            the matrix is not positive definite symmetric.

      if ( *p_Lkk <= 0.0 ) return -1;

//            Otherwise take the square root of the diagonal element.

      *p_Lkk = sqrt( *p_Lkk );
      reciprocal = 1.0 / *p_Lkk;

//            For rows i = k+1 to n-1, column k, calculate the difference
//            between the i,k th element and the inner product of the first
//            k-1 columns of row i and row k, then divide the difference by
//            the diagonal element in row k.
//            Store the transposed element in the upper triangular matrix.

      p_Li0 = p_Lk0 + n;
      for (i = k + 1; i < n; p_Li0 += n, i++) {
         for (p = 0; p < k; p++)
            *(p_Li0 + k) -= *(p_Li0 + p) * *(p_Lk0 + p);
         *(p_Li0 + k) *= reciprocal;
         *(p_Lk0 + i) = *(p_Li0 + k);
      }  
   }
   return 0;
}


////////////////////////////////////////////////////////////////////////////////
//  int Choleski_LU_Solve(double *LU, double *B, double *x,  int n)           //
//                                                                            //
//  Description:                                                              //
//     This routine uses Choleski's method to solve the linear equation       //
//     Ax = B.  This routine is called after the matrix A has been decomposed //
//     into a product of a lower triangular matrix L and an upper triangular  //
//     matrix U which is the transpose of L. The matrix A is the product LU.  //
//     The solution proceeds by solving the linear equation Ly = B for y and  //
//     subsequently solving the linear equation Ux = y for x.                 //
//                                                                            //
//  Arguments:                                                                //
//     double *LU  Pointer to the first element of the matrix whose elements  //
//                 form the lower and upper triangular matrix factors of A.   //
//     double *B   Pointer to the column vector, (n x 1) matrix, B            //
//     double *x   Solution to the equation Ax = B.                           //
//     int     n   The number of rows and/or columns of the matrix LU.        //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix L is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N], B[N], x[N];                                            //
//                                                                            //
//     (your code to create matrix A and column vector B)                     //
//     err = Choleski_LU_Decomposition(&A[0][0], N);                          //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else {                                                                 //
//        err = Choleski_LU_Solve(&A[0][0], B, x, n);                         //
//        if (err < 0) printf(" Matrix A is singular\n");                     //
//        else printf(" The solution is \n");                                 //
//           ...                                                              //
//     }                                                                      //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Choleski_LU_Solve(double *LU, double B[], double x[], int n)
{

//         Solve the linear equation Ly = B for y, where L is a lower
//         triangular matrix.
   
   if ( Lower_Triangular_Solve(LU, B, x, n) < 0 ) return -1;

//         Solve the linear equation Ux = y, where y is the solution
//         obtained above of Ly = B and U is an upper triangular matrix.

   return Upper_Triangular_Solve(LU, x, x, n);
}


////////////////////////////////////////////////////////////////////////////////
//  int Choleski_LU_Inverse(double *LU,  int n)                               //
//                                                                            //
//  Description:                                                              //
//     This routine uses Choleski's method to find the inverse of the matrix  //
//     A.  This routine is called after the matrix A has been decomposed      //
//     into a product of a lower triangular matrix L and an upper triangular  //
//     matrix U which is the transpose of L. The matrix A is the product of   //
//     the L and U.  Upon completion, the inverse of A is stored in LU so     //
//     that the matrix LU is destroyed.                                       //
//                                                                            //
//  Arguments:                                                                //
//     double *LU  On input, the pointer to the first element of the matrix   //
//                 whose elements form the lower and upper triangular matrix  //
//                 factors of A.  On output, the matrix LU is replaced by the //
//                 inverse of the matrix A equal to the product of L and U.   //
//     int     n   The number of rows and/or columns of the matrix LU.        //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix L is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N], B[N], x[N];                                            //
//                                                                            //
//     (your code to create matrix A and column vector B)                     //
//     err = Choleski_LU_Decomposition(&A[0][0], N);                          //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else {                                                                 //
//        err = Choleski_LU_Inverse(&A[0][0], n);                             //
//        if (err < 0) printf(" Matrix A is singular\n");                     //
//        else printf(" The inverse is \n");                                  //
//           ...                                                              //
//     }                                                                      //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Choleski_LU_Inverse(double *LU, int n)
{
   int i, j, k;
   double *p_i, *p_j, *p_k;
   double sum;

   if ( Lower_Triangular_Inverse(LU, n) < 0 ) return -1;
  
//         Premultiply L inverse by the transpose of L inverse.      

   for (i = 0, p_i = LU; i < n; i++, p_i += n) {
      for (j = 0, p_j = LU; j <= i; j++, p_j += n) {
         sum = 0.0;
         for (k = i, p_k = p_i; k < n; k++, p_k += n)
            sum += *(p_k + i) * *(p_k + j);
         *(p_i + j) = sum;
         *(p_j + i) = sum;
      }
   }
  
   return 0;
}
