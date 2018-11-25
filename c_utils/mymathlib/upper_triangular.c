////////////////////////////////////////////////////////////////////////////////
// File: upper_triangular.c                                                   //
// Routines:                                                                  //
//    Upper_Triangular_Solve                                                  //
//    Upper_Triangular_Inverse                                                //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  int Upper_Triangular_Solve(double *U, double *B, double x[], int n)       //
//                                                                            //
//  Description:                                                              //
//     This routine solves the linear equation Ux = B, where U is an n x n    //
//     upper triangular matrix.  (The subdiagonal part of the matrix is       //
//     not addressed.)                                                        //
//     The algorithm follows:                                                 //
//                  x[n-1] = B[n-1]/U[n-1][n-1], and                          //
//     x[i] = [B[i] - (U[i][i+1] * x[i+1]  + ... + U[i][n-1] * x[n-1])]       //
//                                                                 / U[i][i], //
//     for i = n-2, ..., 0.                                                   //
//                                                                            //
//  Arguments:                                                                //
//     double *U   Pointer to the first element of the upper triangular       //
//                 matrix.                                                    //
//     double *B   Pointer to the column vector, (n x 1) matrix, B.           //
//     double *x   Pointer to the column vector, (n x 1) matrix, x.           //
//     int     n   The number of rows or columns of the matrix U.             //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix U is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N], B[N], x[N];                                            //
//                                                                            //
//     (your code to create matrix A and column vector B)                     //
//     err = Upper_Triangular_Solve(&A[0][0], B, x, n);                       //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else printf(" The solution is \n");                                    //
//           ...                                                              //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Upper_Triangular_Solve(double *U, double B[], double x[], int n)
{
   int i, k;

//         Solve the linear equation Ux = B for x, where U is an upper
//         triangular matrix.                                      
   
   for (k = n-1, U += n * (n - 1); k >= 0; U -= n, k--) {
      if (*(U + k) == 0.0) return -1;           // The matrix U is singular
      x[k] = B[k];
      for (i = k + 1; i < n; i++) x[k] -= x[i] * *(U + i);
      x[k] /= *(U + k);
   }

   return 0;
}


////////////////////////////////////////////////////////////////////////////////
//  int Upper_Triangular_Inverse(double *U,  int n)                           //
//                                                                            //
//  Description:                                                              //
//     This routine calculates the inverse of the upper triangular matrix U.  //
//     The subdiagonal part of the matrix is not addressed.                   //
//     The algorithm follows:                                                 //
//        Let M be the inverse of U, then U M = I,                            //
//     M[n-1][n-1] = 1.0 / U[n-1][n-1] and                                    //
//     M[i][j] = -( U[i][i+1] M[i+1][j] + ... + U[i][j] M[j][j] ) / U[i][i],  //
//     for i = n-2, ... , 0,  j = n-1, ..., i+1.                              //
//                                                                            //
//                                                                            //
//  Arguments:                                                                //
//     double *U   On input, the pointer to the first element of the matrix   //
//                 whose upper triangular elements form the matrix which is   //
//                 to be inverted. On output, the upper triangular part is    //
//                 replaced by the inverse.  The subdiagonal elements are     //
//                 not modified.                                              //
//     int     n   The number of rows and/or columns of the matrix U.         //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix U is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double U[N][N];                                                        //
//                                                                            //
//     (your code to create the matrix U)                                     //
//     err = Upper_Triangular_Inverse(&U[0][0], N);                           //
//     if (err < 0) printf(" Matrix U is singular\n");                        //
//     else {                                                                 //
//        printf(" The inverse is \n");                                       //
//           ...                                                              //
//     }                                                                      //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Upper_Triangular_Inverse(double *U, int n)
{
   int i, j, k;
   double *p_i, *p_j, *p_k;
   double sum;

//         Invert the diagonal elements of the upper triangular matrix U.

   for (k = 0, p_k = U; k < n; p_k += (n + 1), k++) {
      if (*p_k == 0.0) return -1;
      else *p_k = 1.0 / *p_k;
   }

//         Invert the remaining upper triangular matrix U.

   for (i = n - 2, p_i = U + n * (n - 2); i >=0; p_i -= n, i-- ) {
      for (j = n - 1; j > i; j--) {
         sum = 0.0;
         for (k = i + 1, p_k = p_i + n; k <= j; p_k += n, k++ ) {
            sum += *(p_i + k) * *(p_k + j);
         }
         *(p_i + j) = - *(p_i + i) * sum;
      }
   }
  
   return 0;
}
