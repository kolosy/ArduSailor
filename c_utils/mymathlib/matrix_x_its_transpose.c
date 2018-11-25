////////////////////////////////////////////////////////////////////////////////
// File: matrix_x_its_transpose.c                                             //
// Routine(s):                                                                //
//    Matrix_x_Its_Transpose                                                  //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Matrix_x_Its_Transpose(double *C, double *A, int nrows, int ncols )  //
//                                                                            //
//  Description:                                                              //
//     Post multiply an nrows x ncols matrix A by its transpose.   The result //
//     is an  nrows x nrows square symmetric matrix C, i.e. C = A A', where ' //
//     denotes the transpose.                                                 //
//     I.e. C = (Cij), where Cij = Sum (Aik Ajk) where the sum extends from   //
//     k = 0 to ncols - 1.                                                    //
//                                                                            //
//     The matrix C should be declared as double C[nrows][nrows] in the       //
//     calling routine.  The memory allocated to C should not include any     //
//     memory allocated to A.                                                 //
//                                                                            //
//  Arguments:                                                                //
//     double *C    Pointer to the first element of the matrix C.             //
//     double *A    Pointer to the first element of the matrix A.             //
//     int    nrows The number of rows of matrix A.                           //
//     int    ncols The number of columns of the matrices A.                  //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     double A[M][N], C[M][M];                                               //
//                                                                            //
//     (your code to initialize the matrix A)                                 //
//                                                                            //
//     Matrix_x_Its_Transpose(&C[0][0], &A[0][0], M, N);                      //
//     printf("The matrix C = AA ' is \n"); ...                               //
////////////////////////////////////////////////////////////////////////////////
void Matrix_x_Its_Transpose(double *C, double *A, int nrows, int ncols)
{
   int i,j,k;
   double *pAi0 = A;
   double *pAj0;
   double *pCi0 = C;
   double *pCji;

   for (i = 0; i < nrows; pCi0 += nrows, pAi0 += ncols, i++) {
      pCji = pCi0 + i;
      pAj0 = pAi0; 
      for (j = i; j < nrows; pCji += nrows, j++) {
         *(pCi0 + j) = 0.0; 
         for (k = 0; k < ncols; k++) *(pCi0 + j) += *(pAi0 + k) * *pAj0++;
         *pCji = *(pCi0 + j);
      }
   }
}
