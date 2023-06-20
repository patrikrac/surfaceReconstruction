/*
Patrik Rác and Jean-Yves Verhaeghe
Project Algorithms and Datastructures
Main algorithm file
*/
#include "point_cloud.hpp"

int main(int argc, char *argv[])
{
 
    if (argc != 2)
    {
        std::cout << "Usage: ./proj <path to LAS file>" << std::endl;
        return -1;
    }
   /*Standard procedure to compute the surface normal vectors of a point cloud*/
   PointCloud pc(argv[1]);
   pc.prepareDirection();
   std::cout << "Building balanced kd-tree." << std::endl;
   pc.kdtree.buildBalanced(pc.points);
   std::cout << "Done." << std::endl;
   std::cout << "Computing the surface normals." << std::endl;
   pc.computeSurfaceNormals(15);
   std::cout << "Done." << std::endl;
   std::cout << "Writing results." << std::endl;
    pc.writePLY("out.ply");
    std::cout << "Done." << std::endl;
    
}