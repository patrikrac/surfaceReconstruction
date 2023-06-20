/*
Patrik Rác and Jean-Yves Verhaeghe
Project Algorithms and Datastructures
Implementation of the surface normal calculation
*/
#include "normal.hpp"

std::vector<double> computeSurfaceNormal(Point &p, std::vector<Point> &pts)
{
   const int k = pts.size();
   Point pointAverage;

   std::vector<double> surface_normal(3);

   /*Compute the average point of the given point set*/
   for(int i = 0; i < k; i++)
   {
      pointAverage += pts[i];
   }
   pointAverage*= (1.0/k);

   //Covariance matrix M
   double M[DIM*DIM];
   //Initialize the matrix with one
   for(int i = 0; i < k; i++)
   {
      for(int k = 0; k < DIM; k++)
      {
         for(int j = 0; j < DIM; j++)
         {
               M[k*DIM + j] = 1.0;
         }
      }
   }

     /*Covariance matrix assembly using BLAS DGER.*/
   {
      int m=DIM, n = DIM;
      double alpha = 1.0;
      int incx = 1, incy=1;
      int lda = m;
      for(int i = 0; i < k; i++)
      {
         Point p = pts[i]-pointAverage;
         dger_(&m, &n, &alpha, p.idx, &incx,
                           p.idx, &incy, M, &lda);
      }
   }

   /*Compute the Eigenvalues and Eigenvectors using DGEEV*/
   {
      char jobvl[] =  "N", jobvr[] = "V";
      int n = DIM, lda = DIM, ldvl = 1, ldvr = DIM, info, lwork = -1;
      double wkopt;
      double* work;
      /* Local arrays */
      double wr[DIM], wi[DIM], vr[DIM*DIM];
      /*Workplace querry*/
      dgeev_( jobvl, jobvr, &n, M, &lda, wr, wi, nullptr, &ldvl, vr, &ldvr,
            &wkopt, &lwork, &info );
               
      lwork = (int)wkopt;
      work = (double*)malloc( lwork*sizeof(double) );
      /*Actual execution of the eigenvalue problem*/
      dgeev_( jobvl, jobvr, &n, M, &lda, wr, wi, nullptr, &ldvl, vr, &ldvr,
               work, &lwork, &info );

      int min_idx = std::distance(wr, std::min_element(wr, wr+DIM));

      for (int i=0; i < DIM; i++)
         surface_normal[i]=(vr[i+ min_idx*ldvr]);
   }

  double scalar_product = surface_normal[X]*p.src[X] + surface_normal[Y]*p.src[Y] + surface_normal[Z]*p.src[Z];
  if(scalar_product < 0)
  {
      //Flip the vector 
      for (int i=0; i < DIM; i++)
         surface_normal[i] = -surface_normal[i];
  }


   return surface_normal;
}
