////////////////////////////////////////////////////////////////////////////////
// File: identity_matrix.c                                                    //
// Routine(s):                                                                //
//    Identity_Matrix                                                         //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Identity_Matrix(double *A, int n)                                    //
//                                                                            //
//  Description:                                                              //
//     Set the square n x n matrix A equal to the identity matrix, i.e.       //
//     A[i][j] = 0 if i != j and A[i][i] = 1.                                 //
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
//     Identity_Matrix(&A[0][0], N);                                          //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Identity_Matrix(double *A, int n)
{
   int i,j;

   for (i = 0; i < n - 1; i++) {
      *A++ = 1.0;
      for (j = 0; j < n; j++) *A++ = 0.0;
   } 
   *A = 1.0;
}
