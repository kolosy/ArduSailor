////////////////////////////////////////////////////////////////////////////////
// File: interchange_rows.c                                                   //
// Routine(s):                                                                //
//    Interchange_Rows                                                        //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Interchange_Rows(double *A, int row1, int row2, int ncols)           //
//                                                                            //
//  Description:                                                              //
//     Interchange the rows 'row1' and 'row2' of the  nrows x ncols matrix A. //
//                                                                            //
//  Arguments:                                                                //
//     double *A    Pointer to the first element of the matrix A.             //
//     int    row1  The row of A which is to be interchanged with row row2.   //
//     int    row2  The row of A which is to be interchanged with row row1.   //
//     int    ncols The number of columns of the matrix A.                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     double A[M][N];                                                        //
//     int i, j;                                                              //
//                                                                            //
//  (your code to initialize the matrix A, the row number i and row number j) //
//                                                                            //
//     if ( (i >= 0) && ( i < M ) && (j > 0) && ( j < M ) )                   //
//        Interchange_Rows(&A[0][0], i, j, N);                                //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Interchange_Rows(double *A, int row1, int row2, int ncols)
{
   int i;
   double *pA1, *pA2;
   double temp;

   pA1 = A + row1 * ncols;
   pA2 = A + row2 * ncols;
   for (i = 0; i < ncols; i++) {
      temp = *pA1;
      *pA1++ = *pA2;
      *pA2++ = temp;
   }
}
