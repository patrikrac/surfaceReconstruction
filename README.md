# Surface Reconstruction from Point Cloud Data using k-d Trees

## Description

Implementation of a program computing the surface normals of a point cloud given by LAS files from the swissSURFACE3D dataset.  
The program uses a k-d tree as an accelerator data structure and uses elemental routines from _BLAS_ and _LAPACK_.

## Compilation

The project comes with a `CMakeLists.txt` that allows for the creation of the build system.  
1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`

### Dependencies

The program depends on the `BLAS`, `LAPACK` and `tinyply` libraries to be installed.

## Execution 

The program can be executed with 

- Usage: `./proj <path to LAS file>`

where a valid path to a LAS file from the swissSURFACE3D is required.  
The program (in default state) creates an `out.ply` file that can then be visualized using tools like `meshlab`.

### Developed by the students 
- Jean-Yves Verhaeghe
- Patrik Rac

