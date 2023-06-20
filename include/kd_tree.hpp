/*
Patrik Rác and Jean-Yves Verhaeghe
Project Algorithms and Datastructures
Header for the KDTree and KDNode classes
*/

#pragma once

#include<memory>
#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>

#include "point.hpp"

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

#define LEFT 0
#define RIGHT 1


class KdNode
{
    /*Point associated with the given Node*/
    Point p;

    /*Left and right children of the given Node*/
     std::shared_ptr<KdNode> left, right;

    /*Current division plane the current node resides on*/
    short plane = X;

public:
    KdNode(const Point &p) :p(p), left(nullptr), right(nullptr)
    {}
    KdNode(const Point &p, const short &plane) :p(p), left(nullptr), right(nullptr), plane(plane)
    {}
    KdNode(const KdNode &node): p(node.p), left(node.left) , right(node.right), plane(node.plane)
    {}

    ~KdNode()
    {}

    /*Query funcitons for the existence of left/right children*/
    bool hasRight() const {return !(right == nullptr);}
    bool hasLeft() const {return !(left == nullptr);}
    bool isLeaf() const {return (left == nullptr) && (right == nullptr);}

    /*Access operations for the Point values of the current node*/
    const Point &getPoint() const {return p;}
    const double &getX() const {return p.x;}
    const double &getY() const {return p.y;}
    const double &getZ() const {return p.z;}

    /*Access operation for the current plane*/
    short &Plane(){return plane;}
    const short &Plane() const {return plane;}

    /*Access operation for the children of the node*/
    std::shared_ptr<KdNode> &Left() {return left;}
    std::shared_ptr<KdNode> &Right() {return right;}
    std::shared_ptr<KdNode> &operator[](const short &index) {
       if(index == LEFT) return left;
       else return right;
    }
    const std::shared_ptr<KdNode> &Left() const {return left;}
    const std::shared_ptr<KdNode> &Right() const {return right;}
    const std::shared_ptr<KdNode> &operator[](const short &index) const  {
       if(index == LEFT) return left;
       else return right;
    }
};

class KdTree
{
    std::shared_ptr<KdNode> root;

public:
    KdTree() :root(nullptr) 
    {}
    KdTree(std::shared_ptr<KdNode> &node) : root(node) 
    {}
    KdTree(Point &p) : root(std::make_shared<KdNode>(p))
    {}

    /*Operations concerning a single node of the tree*/
    void insert(const Point &p);
    std::shared_ptr<KdNode> find(const Point &p);
    std::vector<Point> kNearestNeighbors(const int k, const Point &p);

    /*Function that builds a more balanced tree from a set of points*/
    void buildBalanced(std::vector<Point> points);

private:
    void nearestNeighbor(const std::shared_ptr<KdNode> &node, const Point &p, std::shared_ptr<KdNode> &closest, double &min, const std::unordered_map<Point, bool> &exclude);
    void buildBalanceRecursion(std::vector<Point> &points, int start, int end, short sorting_plane);
};