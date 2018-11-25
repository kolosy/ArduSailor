////////////////////////////////////////////////////////////////////////////////
// File: multiply_matrices.c                                                  //
// Routine(s):                                                                //
//    Multiply_Matrices                                                       //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Multiply_Matrices(double *C, double *A, int nrows, int ncols,        //
//                                                    double *B, int mcols)   //
//                                                                            //
//  Description:                                                              //
//     Post multiply the nrows x ncols matrix A by the ncols x mcols matrix   //
//     B to form the nrows x mcols matrix C, i.e. C = A B.                    //
//     The matrix C should be declared as double C[nrows][mcols] in the       //
//     calling routine.  The memory allocated to C should not include any     //
//     memory allocated to A or B.                                            //
//                                                                            //
//  Arguments:                                                                //
//     double *C    Pointer to the first element of the matrix C.             //
//     double *A    Pointer to the first element of the matrix A.             //
//     int    nrows The number of rows of the matrices A and C.               //
//     int    ncols The number of columns of the matrices A and the           //
//                   number of rows of the matrix B.                          //
//     double *B    Pointer to the first element of the matrix B.             //
//     int    mcols The number of columns of the matrices B and C.            //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     #define NB                                                             //
//     double A[M][N],  B[N][NB], C[M][NB];                                   //
//                                                                            //
//     (your code to initialize the matrices A and B)                         //
//                                                                            //
//     Multiply_Matrices(&C[0][0], &A[0][0], M, N, &B[0][0], NB);             //
//     printf("The matrix C is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Multiply_Matrices(double *C, double *A, int nrows, int ncols,
                                                          double *B, int mcols) 
{
   double *pB;
   double *p_B;
   int i,j,k;

   for (i = 0; i < nrows; A += ncols, i++) 
      for (p_B = B, j = 0; j < mcols; C++, p_B++, j++) {
         pB = p_B;
         *C = 0.0; 
         for (k = 0; k < ncols; pB += mcols, k++) 
            *C += *(A+k) * *pB;
      }
}
