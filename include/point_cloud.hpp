/*
Patrik Rác and Jean-Yves Verhaeghe
Project Algorithms and Datastructures
Header of the point cloud class
*/
#pragma once

#include<iostream>
#include<fstream>
#include<vector>

#include<omp.h>
#include "tinyply.h"

#include "point.hpp"
#include "kd_tree.hpp"
#include "normal.hpp"

#define WindowSize 10000

class PointCloud 
{
public:
    int n_points = 0; /*Number of points*/
    std::vector<Point> points; /*Vector containing all points*/
    KdTree kdtree; /*KdTree allowing for fast processing of the points*/
    std::vector<std::vector<double>> normals; /*Surface Normals*/

    /*Constructor with LAS file reader*/
    PointCloud() = default;
    PointCloud(const char *filename);

    /*Funciton that pre-processes the flight data read from the LAS file and prepares the src point dor the normal computation*/
    void prepareDirection();

    /*Main computation function for the Surface normals.*/
    void computeSurfaceNormals(const int &n);
    /*Writer functions for the VTK and PLY file format*/
    void writeVTK(const char *filename, const bool set_surface_normal=true);
    void writePLY(const char *filename);

private:
    void processPointWindow(const int l, const int r, const int point_l, const int point_r);
};