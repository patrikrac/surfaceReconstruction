/*
Patrik Rác and Jean-Yves Verhaeghe
Project Algorithms and Datastructures
Implementation of the Point class methods
*/
#include "point.hpp"

Point &Point::operator+=(const Point &p)
{
    x+=p.x; y+=p.y; z+=p.z;
    return *this;
}

Point &Point::operator-=(const Point &p)
{
    x-=p.x; y-=p.y; z-=p.z;
    return *this;
}

Point &Point::operator*=(const double &a)
{
    x*=a; y*=a; z*=a;
    return *this;
}

Point Point::operator+(const Point &p)
{
    return Point(x+p.x, y+p.y, z+p.z);
}

Point Point::operator-(const Point &p)
{
    return Point(x-p.x, y-p.y, z-p.z);
}

double Dist(const Point &A, const Point &B)
{
    return sqrt((A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z));
}

double Dist2(const Point &A, const Point &B)
{
    return (A.x-B.x)*(A.x-B.x) + (A.y-B.y)*(A.y-B.y) + (A.z-B.z)*(A.z-B.z);
}