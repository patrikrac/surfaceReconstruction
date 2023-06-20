/*
Patrik Rác and Jean-Yves Verhaeghe
Project Algorithms and Datastructures
Implementation of the K-D tree algorithms
*/
#include "kd_tree.hpp"

void KdTree::insert(const Point &p)
{
    std::shared_ptr<KdNode> current = root;
    std::shared_ptr<KdNode> drag = nullptr;    
    short child_direction = LEFT;

    while (current != nullptr)
    {
        drag = current;
        if(current->getPoint()[current->Plane()] > p[current->Plane()])
        {
            current = current->Left();
            child_direction = LEFT;
        }
        else 
        {
            current = current->Right();
            child_direction = RIGHT;
        }
    }

    if (drag != nullptr)
    {
        short plane = (drag->Plane() + 1)%3;
        (*drag)[child_direction] = std::make_shared<KdNode>(p,plane);
    }
    else
    {
        root = std::make_shared<KdNode>(p);
    }
}

std::shared_ptr<KdNode> KdTree::find(const Point &p)
{
    std::shared_ptr<KdNode> current = root;
    while (current != nullptr)
    {
        if(current->getPoint() == p)
        {
            return current;
        }

        if(current->getPoint()[current->Plane()] > p[current->Plane()])
        {   
            current = current->Left();
            std::cout << "Left" << std::endl;
        }
        else 
        {
            current = current->Right();
            std::cout << "Right" << std::endl;
        }
    }

    return nullptr;
}

std::vector<Point> KdTree::kNearestNeighbors(const int k, const Point &p)
{
    std::vector<Point> closest_list;

    std::unordered_map<Point, bool> exclude;
    exclude.insert({p, true});

    for (int i = 0; i < k; i++)
    {
        std::shared_ptr<KdNode> closest = root;
        double min = Dist(p, root->getPoint());
        nearestNeighbor(root, p, closest, min, exclude);
        exclude.insert({closest->getPoint(), true});
        closest_list.push_back(closest->getPoint());
    }
    
    return closest_list;
}

/*Recursive nearest neighbor computiation; Ignoring all points specified in exclude.*/
void KdTree::nearestNeighbor(const std::shared_ptr<KdNode> &node, const Point &p, std::shared_ptr<KdNode> &closest, 
                                            double &min, const std::unordered_map<Point, bool> &exclude)
{
    const bool consider_node = (exclude.find(node->getPoint()) == exclude.end());

    if(consider_node && node->isLeaf()) 
    {
        const double dist = Dist(p, node->getPoint());
        if(dist < min)
        {
            min = dist;
            closest = node;
        }
    }
    else {
        const short spliting_direction = node->Plane();
        if(p[spliting_direction] < node->getPoint()[spliting_direction])
        {
            //search left
            if(node->hasLeft())
                nearestNeighbor(node->Left(), p, closest, min, exclude);
            

            const double dist = Dist(p,node->getPoint());
            if(consider_node && dist < min)
            {
                min = dist;
                closest = node;
            }
            
            if(node->hasRight() && p[spliting_direction] + min >= node->getPoint()[spliting_direction])
            {
                nearestNeighbor(node->Right(), p, closest, min, exclude);
                if(consider_node && dist < min)
                {
                    min = dist;
                    closest = node;
                }
            }
            
        }
        else
        {
            //search right
            if(node->hasRight())
                nearestNeighbor(node->Right(), p, closest, min, exclude);

            const double dist = Dist(p,node->getPoint());
            if(consider_node && dist < min)
            {
                min = dist;
                closest = node;
            }
            
            if(node->hasLeft() && p[spliting_direction] - min <= node->getPoint()[spliting_direction])
            {
                nearestNeighbor(node->Left(), p, closest, min, exclude);
                if(consider_node && dist < min)
                {
                    min = dist;
                    closest = node;
                }
            }
            
        }       
    }
}


/*Recursive method that assembles the KDTree from a set of points and ensures balance of the tree*/
void KdTree::buildBalanced(std::vector<Point> points)
{
    buildBalanceRecursion(points, 0, points.size(), X);
}

void KdTree::buildBalanceRecursion(std::vector<Point> &points, int start, int end, short sorting_plane)
{ 
    if(start > end-1) return; 
    if(start == end-1)
    {
        this->insert(points[start]);
        return;
    }
    
    switch(sorting_plane)  
    {
        case X:  std::sort(points.begin()+start, points.begin()+end, [](Point &a, Point &b) {return a.x > b.x;}); break;
        case Y:  std::sort(points.begin()+start, points.begin()+end, [](Point &a, Point &b) {return a.y > b.y;}); break;
        case Z:  std::sort(points.begin()+start, points.begin()+end, [](Point &a, Point &b) {return a.z > b.z;}); break;
    }

    int med = start + (end-start)/2;
    sorting_plane = (sorting_plane + 1)%3;
    this->insert(points[med]);
    buildBalanceRecursion(points, start, med, sorting_plane);
    buildBalanceRecursion(points, med+1, end, sorting_plane);
}