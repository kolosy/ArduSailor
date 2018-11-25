////////////////////////////////////////////////////////////////////////////////
// File: lower_triangular.c                                                   //
// Routines:                                                                  //
//    Lower_Triangular_Solve                                                  //
//    Lower_Triangular_Inverse                                                //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  int Lower_Triangular_Solve(double *L, double *B, double x[], int n)       //
//                                                                            //
//  Description:                                                              //
//     This routine solves the linear equation Lx = B, where L is an n x n    //
//     lower triangular matrix.  (The superdiagonal part of the matrix is     //
//     not addressed.)                                                        //
//     The algorithm follows:                                                 //
//                      x[0] = B[0]/L[0][0], and                              //
//     x[i] = [B[i] - (L[i][0] * x[0]  + ... + L[i][i-1] * x[i-1])] / L[i][i],//
//     for i = 1, ..., n-1.                                                   //
//                                                                            //
//  Arguments:                                                                //
//     double *L   Pointer to the first element of the lower triangular       //
//                 matrix.                                                    //
//     double *B   Pointer to the column vector, (n x 1) matrix, B.           //
//     double *x   Pointer to the column vector, (n x 1) matrix, x.           //
//     int     n   The number of rows or columns of the matrix L.             //
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
//     err = Lower_Triangular_Solve(&A[0][0], B, x, n);                       //
//     if (err < 0) printf(" Matrix A is singular\n");                        //
//     else printf(" The solution is \n");                                    //
//           ...                                                              //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Lower_Triangular_Solve(double *L, double B[], double x[], int n)
{
   int i, k;

//         Solve the linear equation Lx = B for x, where L is a lower
//         triangular matrix.                                      
   
   for (k = 0; k < n; L += n, k++) {
      if (*(L + k) == 0.0) return -1;           // The matrix L is singular
      x[k] = B[k];
      for (i = 0; i < k; i++) x[k] -= x[i] * *(L + i);
      x[k] /= *(L + k);
   }

   return 0;
}


////////////////////////////////////////////////////////////////////////////////
//  int Lower_Triangular_Inverse(double *L,  int n)                           //
//                                                                            //
//  Description:                                                              //
//     This routine calculates the inverse of the lower triangular matrix L.  //
//     The superdiagonal part of the matrix is not addressed.                 //
//     The algorithm follows:                                                 //
//        Let M be the inverse of L, then L M = I,                            //
//     M[i][i] = 1.0 / L[i][i] for i = 0, ..., n-1, and                       //
//     M[i][j] = -[(L[i][j] M[j][j] + ... + L[i][i-1] M[i-1][j])] / L[i][i],  //
//     for i = 1, ..., n-1, j = 0, ..., i - 1.                                //
//                                                                            //
//                                                                            //
//  Arguments:                                                                //
//     double *L   On input, the pointer to the first element of the matrix   //
//                 whose lower triangular elements form the matrix which is   //
//                 to be inverted. On output, the lower triangular part is    //
//                 replaced by the inverse.  The superdiagonal elements are   //
//                 not modified.                                              //
//                 its inverse.                                               //
//     int     n   The number of rows and/or columns of the matrix L.         //
//                                                                            //
//  Return Values:                                                            //
//     0  Success                                                             //
//    -1  Failure - The matrix L is singular.                                 //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double L[N][N];                                                        //
//                                                                            //
//     (your code to create the matrix L)                                     //
//     err = Lower_Triangular_Inverse(&L[0][0], N);                           //
//     if (err < 0) printf(" Matrix L is singular\n");                        //
//     else {                                                                 //
//        printf(" The inverse is \n");                                       //
//           ...                                                              //
//     }                                                                      //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Lower_Triangular_Inverse(double *L, int n)
{
   int i, j, k;
   double *p_i, *p_j, *p_k;
   double sum;

//         Invert the diagonal elements of the lower triangular matrix L.

   for (k = 0, p_k = L; k < n; p_k += (n + 1), k++) {
      if (*p_k == 0.0) return -1;
      else *p_k = 1.0 / *p_k;
   }

//         Invert the remaining lower triangular matrix L row by row.

   for (i = 1, p_i = L + n; i < n; i++, p_i += n) {
      for (j = 0, p_j = L; j < i; p_j += n, j++) {
         sum = 0.0;
         for (k = j, p_k = p_j; k < i; k++, p_k += n)
            sum += *(p_i + k) * *(p_k + j);
         *(p_i + j) = - *(p_i + i) * sum;
      }
   }
  
   return 0;
}
