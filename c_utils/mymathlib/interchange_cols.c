////////////////////////////////////////////////////////////////////////////////
// File: interchange_cols.c                                                   //
// Routine(s):                                                                //
//    Interchange_Columns                                                     //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Interchange_Columns(double *A, int col1, int col2, int nrows,        //
//                                                                 int ncols) //
//                                                                            //
//  Description:                                                              //
//     Interchange the columns 'col1' and 'col2' of the  nrows x ncols        //
//     matrix A.                                                              //
//                                                                            //
//  Arguments:                                                                //
//     double *A    Pointer to the first element of the matrix A.             //
//     int    col1  The column of A which is to be interchanged with col2.    //
//     int    col2  The column of A which is to be interchanged with col1.    //
//     int    nrows The number of rows matrix A.                              //
//     int    ncols The number of columns of the matrix A.                    //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     #define M                                                              //
//     double A[M][N];                                                        //
//     int i,j;                                                               //
//                                                                            //
//     (your code to initialize the matrix A, the column number i and column  //
//       number j)                                                            //
//                                                                            //
//     if ( (i >= 0) && ( i < N ) && ( j >= 0 ) && (j < N) )                  //
//        Interchange_Columns(&A[0][0], i, j, M, N);                          //
//     printf("The matrix A is \n"); ...                                      //
////////////////////////////////////////////////////////////////////////////////
void Interchange_Columns(double *A, int col1, int col2, int nrows, int ncols)
{
   int i;
   double *pA1, *pA2;
   double temp;

   pA1 = A + col1;
   pA2 = A + col2;
   for (i = 0; i < nrows; pA1 += ncols, pA2 += ncols, i++) {
      temp = *pA1;
      *pA1 = *pA2;
      *pA2 = temp;
   }
}
