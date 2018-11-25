////////////////////////////////////////////////////////////////////////////////
// File: transpose_square_matrix.c                                            //
// Routine(s):                                                                //
//    Transpose_Square_Matrix                                                 //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Transpose_Square_Matrix( double *A, int n )                          //
//                                                                            //
//  Description:                                                              //
//     Take the transpose of A and store in place.                            //
//                                                                            //
//  Arguments:                                                                //
//     double *A    Pointer to the first element of the matrix A.             //
//     int    n     The number of rows and columns of the matrix A.           //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N];                                                        //
//                                                                            //
//     (your code to initialize the matrix A)                                 //
//                                                                            //
//     Transpose_Square_Matrix( &A[0][0], N);                                 //
//     printf("The transpose of A is \n"); ...                                //
////////////////////////////////////////////////////////////////////////////////
void Transpose_Square_Matrix( double *A, int n ) 
{
   double *pA, *pAt;
   double temp;
   int i,j;

   for (i = 0; i < n; A += n + 1, i++) {
      pA = A + 1;
      pAt = A + n;
      for (j = i+1 ; j < n; pA++, pAt += n, j++) {
         temp = *pAt;
         *pAt = *pA;
         *pA = temp;
      } 
   }
}
