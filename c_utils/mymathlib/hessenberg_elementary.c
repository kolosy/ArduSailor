////////////////////////////////////////////////////////////////////////////////
// File: hessenberg_elementary.c                                              //
// Routine(s):                                                                //
//    Hessenberg_Form_Elementary                                              //
//    Hessenberg_Elementary_Transform                                         //
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>                      // required for malloc() and free()    
#include <math.h>                        // required for fabs()

//                    Required Externally Defined Routines 
void Interchange_Rows(double *A, int row1, int row2, int ncols);
void Interchange_Columns(double *A, int col1, int col2, int nrows, int ncols);
void Identity_Matrix(double *A, int n);
void Copy_Vector(double *d, double *s, int n);

//                        Internally Defined Routines 
static void Hessenberg_Elementary_Transform(double* H, double *S,
                                                            int perm[], int n);

////////////////////////////////////////////////////////////////////////////////
//  int Hessenberg_Form_Elementary(double *A, double *S, int n)               //
//                                                                            //
//  Description:                                                              //
//     This program transforms the square matrix A to a similar matrix in     //
//     Hessenberg form by a multiplying A on the right by a sequence of       //
//     elementary transformations and on the left by the sequence of inverse  //
//     transformations.                                                       //
//     Def:  Two matrices A and B are said to be similar if there exists a    //
//           nonsingular matrix S such that A S = S B.                        //
//     Def   A Hessenberg matrix is the sum of an upper triangular matrix and //
//           a matrix all of whose components are 0 except possibly on its    //
//           subdiagonal.  A Hessenberg matrix is sometimes said to be almost //
//           upper triangular.                                                //
//     The algorithm proceeds by successivly selecting columns j = 0,...,n-3  //
//     and then assuming that columns 0, ..., j-1 have been reduced to Hessen-//
//     berg form, for rows j+1 to n-1, select that row j' for which |a[j'][j]|//
//     is a maximum and interchange rows j+1 and j' and columns j+1 and j'.   //
//     Then for each i = j+2 to n-1, let x = a[i][j] / a[j+1][j] subtract     //
//     x * row j+1 from row i and add x * column i to column j+1.             //
//                                                                            //
//  Arguments:                                                                //
//     double *A     On input a pointer to the first element of the matrix    //
//                   A[n][n].  The matrix A is replaced with the matrix H,    //
//                   a matrix in Hessenberg form similar to A.                //
//     double *S     On output the transform such that A S = S H.             //
//                   The matrix S should be dimensioned at least n x n in the //
//                   calling routine.                                         //
//     int    n      The number of rows or columns of the matrix A.           //
//                                                                            //
//  Return Values:                                                            //
//      0  Success                                                            //
//     -1  Failure - Not enough memory                                        //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double A[N][N], S[N][N];                                               //
//                                                                            //
//     (your code to create the matrix A)                                     //
//     if (Hessenberg_Form_Elementary(&A[0][0], (double*)S, N ) < 0) {        //
//        printf("Not enough memory\n"); exit(0);                             //
//     }                                                                      //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
int Hessenberg_Form_Elementary(double *A, double* S, int n)
{
   int i, j, k, col, row;
   int* perm;
   double *p_row, *pS_row;
   double max;
   double s;
   double *pA, *pB, *pC, *pS;

         // n x n matrices for which n <= 2 are already in Hessenberg form

   if (n <= 1) { *S = 1.0; return 0; }
   if (n == 2) { *S++ = 1.0; *S++ = 0.0; *S++ = 1.0; *S = 0.0; return 0; }

                       // Allocate working memory

   perm = (int*) malloc(n * sizeof(int));
   if (perm == NULL) return -1;             // not enough memory

           // For each column use Elementary transformations 
           //   to zero the entries below the subdiagonal.

   p_row = A + n;
   pS_row = S + n;
   for (col = 0; col < (n - 2); p_row += n, pS_row += n, col++) {

          // Find the row in column "col" with maximum magnitude where 
          // row >= col + 1.                
     
      row = col + 1;
      perm[row] = row;
      for (pA = p_row + col, max = 0.0, i = row; i < n; pA += n, i++)
         if (fabs(*pA) > max) { perm[row] = i; max = fabs(*pA); }
      
          // If perm[row] != row, then interchange row "row" and row
          // perm[row] and interchange column "row" and column perm[row].

      if ( perm[row] != row ) {
         Interchange_Rows(A, row, perm[row], n);
         Interchange_Columns(A, row, perm[row], n, n);
      }
         
          // Zero out the components lying below the subdiagonal.

      pA = p_row + n;
      pS = pS_row + n;
      for (i = col + 2; i < n; pA += n, pS += n, i++) {
         s = *(pA + col) / *(p_row + col);
         for (j = 0; j < n; j++) 
            *(pA + j) -= *(p_row + j) * s;
         *(pS + col) = s;
         for (j = 0, pB = A + col + 1, pC = A + i; j < n; pB +=n, pC += n, j++) 
            *pB += s * *pC;
      }
   }
   pA = A + n + n;
   pS = S + n + n;
   for (i = 2; i < n; pA += n, pS += n, i++) Copy_Vector(pA, pS, i - 1);

   Hessenberg_Elementary_Transform(A, S, perm, n);
   
   free(perm);
   return 0;
}


////////////////////////////////////////////////////////////////////////////////
//  static void Hessenberg_Elementary_Transform(double* H, double *S,         //
//                                                        int perm[], int n)  //
//                                                                            //
//  Description:                                                              //
//     Given a n x n matrix A, let H be the matrix in Hessenberg form similar //
//     to A, i.e. A S = S H.  If v is an eigenvector of H with eigenvalue z,  //
//     i.e. Hv = zv, then ASv = SHv = z Sv, i.e. Sv is the eigenvector of A   //
//     with corresponding eigenvalue z.                                       //
//     This routine returns S where S is the similarity transformation such   //
//     that A S = S H.                                                        //
//                                                                            //
//  Arguments:                                                                //
//     double* H     On input a matrix in Hessenberg form with transformation //
//                   elements stored below the subdiagonal part.              //
//                   On output the matrix in Hessenberg form with elements    //
//                   below the subdiagonal zeroed out.                        //
//     double* S     On output, the transformations matrix such that          //
//                   A S = S H.                                               //
//     int    perm[] Array of row/column interchanges.                        //
//     int    n      The order of the matrices H and S.                       //
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
//                                                                            //
static void Hessenberg_Elementary_Transform(double *H, double* S, int perm[],
                                                                         int n)
{
   int i, j;
   double *pS, *pH;
   double x;

   Identity_Matrix(S, n);
   for (i = n - 2; i >= 1; i--) {
      pH = H + n * (i + 1);
      pS = S + n * (i + 1);
      for (j = i + 1; j < n; pH += n, pS += n, j++) {
         *(pS + i) = *(pH + i - 1);
         *(pH + i - 1) = 0.0;
      }
      if (perm[i] != i) {
         pS = S + n * i;
         pH = S + n * perm[i];
         for (j = i; j < n; j++) {
            *(pS + j) = *(pH + j);
            *(pH + j) = 0.0;
         }
         *(pH + i) = 1.0;
      }
   }
}
