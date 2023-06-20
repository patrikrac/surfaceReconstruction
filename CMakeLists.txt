cmake_minimum_required( VERSION 3.10)

set (CMAKE_CXX_STANDARD 11)

project( proj LANGUAGES CXX)

SET( APP_EXE proj)

ADD_EXECUTABLE ( ${APP_EXE} 
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    ${CMAKE_SOURCE_DIR}/src/point.cpp
    ${CMAKE_SOURCE_DIR}/src/point_cloud.cpp
    ${CMAKE_SOURCE_DIR}/src/kd_tree.cpp
    ${CMAKE_SOURCE_DIR}/src/normal.cpp
    )

target_include_directories(${APP_EXE} PRIVATE  
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/include
    ) 

find_package(tinyply REQUIRED)
find_package(BLAS REQUIRED) 
find_package(LAPACK REQUIRED) 
target_link_libraries(proj PRIVATE BLAS::BLAS)
target_link_libraries(proj PRIVATE LAPACK::LAPACK)
target_link_libraries(proj PRIVATE tinyply)

find_package(OpenMP)
target_link_libraries(proj PRIVATE OpenMP::OpenMP_CXX)

target_link_libraries(proj PRIVATE m)

find_library(TINYPLY_LIB tinyply REQUIRED)
target_link_libraries(proj PRIVATE "${TINYPLY_LIB}")

target_compile_options(${APP_EXE}
    PRIVATE -O3 
)
