/*
Patrik Rác and Jean-Yves Verhaeghe
Project Algorithms and Datastructures
Header for the Point class
*/
#pragma once
#include <cmath>
#include <vector>

#define X 0
#define Y 1
#define Z 2

class Point
{
public:

  /*Coordinates of the point*/
    union {
		struct {
			double x , y, z;
		};
		double idx[3];
    };

  /*Scan angle and GPS time associated with the point*/
  short scan_angle = 0;
  double gps_time = 0.0;

  /*Direction vector of the laser*/
  std::vector<double> src = {0,0,0};
  bool confident = false;

    Point(): x(0), y(0), z(0) {};  
    Point(double x, double y, double z): x(x), y(y), z(z) {};
    Point(double x, double y, double z, short scan_angle): x(x), y(y), z(z), scan_angle(scan_angle) {};  
    Point(double x, double y, double z, short scan_angle, double gps_time): x(x), y(y), z(z), scan_angle(scan_angle), gps_time(gps_time) {};  

    double &operator[](const short &index) {return idx[index];}
    const double &operator[](const short &index) const {return idx[index];}

    Point &operator+=(const Point &p);
    Point &operator-=(const Point &p);
    Point &operator*=(const double &a);

    Point operator+(const Point &p);
    Point operator-(const Point &p);

    const bool operator==(const Point &p) const
    {
      if(x == p.x && y==p.y && z==p.z) return true;
      return false;
    }
};

//Hash funciton for the Point class (very generic)
template<>
struct std::hash<Point>
{
    std::size_t operator()(Point const& p) const noexcept
    {
        return p.x; 
    }
};

/*Distance*/
double Dist(const Point &A, const Point &B);
/*Squared distance*/
double Dist2(const Point &A, const Point &B);