////////////////////////////////////////////////////////////////////////////////
// File: copy_vector.c                                                        //
// Routine(s):                                                                //
//    Copy_Vector                                                             //
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  void Copy_Vector(double *d, double *s, int n)                             //
//                                                                            //
//  Description:                                                              //
//     Copy the n dimensional vector s(source) to the n dimensional           //
//     vector d(destination).  The memory locations of the source and         //
//     destination vectors must not overlap, otherwise the results            //
//     are installation dependent.                                            //
//                                                                            //
//  Arguments:                                                                //
//      double *d  Pointer to the first element of the destination vector d.  //
//      double *s  Pointer to the first element of the source vector s.       //
//      int    n   The number of elements of the source / destination vectors.//
//                                                                            //
//  Return Values:                                                            //
//     void                                                                   //
//                                                                            //
//  Example:                                                                  //
//     #define N                                                              //
//     double v[N],  vd[N];                                                   //
//                                                                            //
//     (your code to initialize the vector v)                                 //
//                                                                            //
//     Copy_Vector(vd, v, N);                                                 //
//     printf(" Vector vd is \n");                                            //
////////////////////////////////////////////////////////////////////////////////

#include <string.h>                                 // required for memcpy()

void Copy_Vector(double *d, double *s, int n)
{
   memcpy(d, s, sizeof(double) * n);
}
