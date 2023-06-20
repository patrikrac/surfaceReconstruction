/*
Patrik Rác and Jean-Yves Verhaeghe
Project Algorithms and Datastructures
Header for the normal calulation file
*/
#pragma once

#include<stdlib.h>
#include<stdio.h>
#include<iostream>
#include <algorithm>
#include<vector>

#include "point.hpp"

#define DIM 3

/*Use LAPACK Routines for Linear Algebra*/

/* SGEEV prototype */
extern "C" {
    extern void dgeev_( char* jobvl, char* jobvr, int* n, double* a,
                    int* lda, double* wr, double* wi, double* vl, int* ldvl,
                    double* vr, int* ldvr, double* work, int* lwork, int* info );
}

/*DGER prototype*/
extern "C" {
    extern void dger_(int *m, int *n, double *alpha, double *x, int *incx,
                        double *y, int *incy, double *a, int *lda);
}

/*Function that uses LAPACK and BLAS to compute the surface normal of point p with neighborhood pts*/
std::vector<double> computeSurfaceNormal(Point &p, std::vector<Point> &pts);
