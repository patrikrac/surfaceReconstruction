/*
Patrik Rác and Jean-Yves Verhaeghe
Project Algorithms and Datastructures
Implementation of the point cloud class
*/
#include "point_cloud.hpp"


//Reads a Lidar file into the point cloud object.
PointCloud::PointCloud(const char *filename)
{
    /*Read important parts of the header*/
    std::ifstream is (filename, std::ifstream::binary);
    //char *header = new char[4];
    char header[4];
    is.read(header, 4);
    std::cout << "Reading " << header << " file." << std::endl;

    is.ignore(2);

    unsigned short global_encoding_bit[1];
    is.read((char *) global_encoding_bit, 2);
    if(*global_encoding_bit != 1)
    {
        std::cerr << "Global encoding bit not set. GPS time required." << std::endl;
    }

    is.ignore(88);

    unsigned long offset_to_pointdata[1]; 
    is.read((char *) offset_to_pointdata, 4);

    is.ignore(5);

    unsigned short offset_data_record[1];
    is.read((char *) offset_data_record, 2);

    unsigned long number_point_records[1];
    is.read((char *) number_point_records, 4);
    std::cout << "Reading " << *number_point_records << " points." << std::endl;

    /*Reserve the space for the number of points*/
    n_points = *number_point_records;
    points.resize(n_points);

    is.ignore(20);

    double x_scale_factor[1], y_scale_factor[1], z_scale_factor[1];
    double x_offset[1], y_offset[1], z_offset[1];

    is.read((char *) x_scale_factor, 8);    is.read((char *) y_scale_factor, 8);    is.read((char *) z_scale_factor, 8);
    is.read((char *) x_offset, 8);    is.read((char *) y_offset, 8);    is.read((char *) z_offset, 8);

    /*Read the points*/
    is.seekg(*offset_to_pointdata, std::ios_base::beg);
    long x[1], y[1], z[1];
    char scan_angle_rank[1];
    double gps_time[1];
    for(int i = 0; i < n_points; i++)
    {
        is.read((char *) x, 4);
        is.read((char *) y, 4);
        is.read((char *) z, 4);

        is.ignore(4);

        is.read((char *) scan_angle_rank, 1);

        is.ignore(3);
        is.read((char *) gps_time, 8);

        is.ignore(*offset_data_record - 28);

        points[i] = Point(((double) *x * (*x_scale_factor)),
                                                ((double) *y * (*y_scale_factor)),
                                                ((double) *z * (*z_scale_factor)),
                                                (short) *scan_angle_rank,
                                                *gps_time);
    }

    std::cout << "Done." << std::endl;
}

void PointCloud::processPointWindow(const int l, const int r, int point_l, int point_r)
{
    if(point_l == point_r)
    {
        /*Alternative; Choose l and r as reference points*/
        point_l = l;
        point_r = r;    
    }

    /*Compute the direction for all points inside the current window*/
    double window_direction_x =  points[point_l].x - points[point_r].x;
    double window_direction_y =  points[point_l].y - points[point_r].y;

    /*Normalize the vector*/
    double l_dir = sqrt(window_direction_x*window_direction_x + window_direction_y*window_direction_y); 
    window_direction_x /= l_dir; window_direction_y /= l_dir;

    /*Iterate trough all points of the window and compute the corresponding surface normal.*/
    for (int i = l; i <= r; i++)
    {
            /*Compute source vector*/
            points[i].src[X] = window_direction_y * sin(points[i].scan_angle * M_PI*1.0/180.0);
            points[i].src[Y] = -window_direction_x * sin(points[i].scan_angle * M_PI*1.0/180.0);
            points[i].src[Z] = cos(points[i].scan_angle * M_PI*1.0/180.0);

            /*Normalize the source vector*/
            double l_src = sqrt(points[i].src[X]*points[i].src[X] + points[i].src[Y]*points[i].src[Y] + points[i].src[Z]*points[i].src[Z]);
            points[i].src[X] /= l_src;  points[i].src[Y] /= l_src;  points[i].src[Z] /= l_src;
    }
}


void PointCloud::prepareDirection()
{
    std::cout << "Preparing source direction." << std::endl;
    
    /*Sorting the points according to the set gps-time*/
    std::sort(points.begin(), points.end(), [](Point &a, Point &b) {return a.gps_time < b.gps_time;});

    /*Variables managing loop behaviour*/
    short angle_difference;

    double direction_x, direction_y;

    int windowLeft = 0, windowRight = 0;
    int indexMinRankLeft = 0, indexMinRankRight = 0;
    short minAngle = 90;
    bool potentialEdge = false;

    while(true)
    {
        if(windowRight >= points.size()-1)
        {
            /*Reached end of the array => Process and finsish*/
            processPointWindow(windowLeft, windowRight, indexMinRankLeft, indexMinRankRight);
            break;
        }

        if(windowRight-windowLeft >= WindowSize)
        {
            /*Maximum window size has been reached ... update points*/
            processPointWindow(windowLeft, windowRight, indexMinRankLeft, indexMinRankRight);
            windowLeft = windowRight+1;
            windowRight = windowLeft;
            continue;
        }

        angle_difference = points[windowRight].scan_angle - points[windowRight+1].scan_angle;
        if(std::abs(angle_difference) > 1)
        {
            /*Assume we reached end of line ... update points*/
            processPointWindow(windowLeft, windowRight, indexMinRankLeft, indexMinRankRight);
            windowLeft = windowRight+1;
            windowRight = windowLeft;
            continue;
        }

        /*Compute the local direction vector between this and the previous point*/         
        double local_direction_x = points[windowRight+1].x - points[windowRight].x;
        double local_direction_y = points[windowRight+1].y - points[windowRight].y;
        
        if(windowLeft != windowRight)
        {
            /*Compute the dot product with the previous point to check direction*/
            double dot = local_direction_x * direction_x + local_direction_y * direction_y;

            direction_x = local_direction_x;
            direction_y = local_direction_y;

            if(dot < 0)
            {
                /*Not consistent mark as *possibly an edge* and do not update the direction fields */
                if(potentialEdge)
                {
                    /*If potential edge was already reached -> Update points and reset to last point*/
                    processPointWindow(windowLeft, windowRight-2, indexMinRankLeft, indexMinRankRight);
                    potentialEdge = false;
                    windowLeft = windowRight -1;
                    continue;
                }
                else
                {
                    potentialEdge = true;
                    continue;
                }
            }
        }

        /*Indications suggest point is in the same scan line -> Process normally*/
        if(std::abs(points[windowRight].scan_angle) < std::abs(minAngle))
        {
            minAngle = points[windowRight].scan_angle;
            indexMinRankLeft = indexMinRankRight = windowRight;
        }
        else if(points[windowLeft].scan_angle == minAngle)
        {
             indexMinRankRight = windowRight;
        }

        windowRight++;
    }
    std::cout << "Done." << std::endl;
}


void PointCloud::computeSurfaceNormals(const int &n)
{
    normals.resize(points.size());

#pragma omp parallel for schedule(dynamic, 10000)
    for(int i = 0; i < points.size(); i++)
    {
        std::vector<Point> neighborhood = kdtree.kNearestNeighbors(n, points[i]);
        normals[i] = computeSurfaceNormal(points[i], neighborhood);
    }
}

/*Function that writes the results into a legacy VTK file*/
void PointCloud::writeVTK(const char *filename, const bool set_surface_normal)
{
    std::ofstream os(filename);
    
    /*Write the header of the VTK legacy file format*/
    os << "# vtk DataFile Version 2.0" << std::endl;
    os << "Lidar Data" << std::endl;
    os << "ASCII" << std::endl;

    os << "DATASET UNSTRUCTURED_GRID" << std::endl <<std::endl;

    /*Print the point data*/
    os << "POINTS " << points.size() << " double" << std::endl;
    for (const auto &p : points)
    {
        os << p.x << " " << p.y << " " << p.z << std::endl;
    }

    if(set_surface_normal)
    {
        /*Print the surface normal vectors*/
        os << "POINT_DATA " << points.size() << std::endl;
        os << "NORMALS normals double" << std::endl;
        for (const auto &n : normals)
        {
            os << n[0] << " " << n[1] << " " << n[2] << std::endl;
        }
    }

    os.close();
}

/*Extractor funcions for normals and points */
static std::vector<double> extractRawData(std::vector<std::vector<double>> &normals)
{
    std::vector<double> raw_normals(3*normals.size());
    for(int i = 0; i < normals.size(); i++)
    {
        raw_normals[3*i + X] = normals[i][X];
        raw_normals[3*i + Y] = normals[i][Y];
        raw_normals[3*i + Z] = normals[i][Z];
    }

    return raw_normals;
}

static std::vector<double> extractRawData(std::vector<Point> &pts)
{
    std::vector<double> raw_pts(3*pts.size());
    for(int i = 0; i < pts.size(); i++)
    {
        raw_pts[3*i + X] = pts[i][X];
        raw_pts[3*i + Y] = pts[i][Y];
        raw_pts[3*i + Z] = pts[i][Z];
    }
    return raw_pts;
}

/*PLY writer using tinyply*/
void PointCloud::writePLY(const char *filename)
{
    std::filebuf fb;
    fb.open(filename, std::ios::out);

    std::ostream outstream(&fb);
    if (outstream.fail()) throw std::runtime_error("failed to open " + std::string(filename));

    tinyply::PlyFile ofile;

    std::vector<double> raw_pts = extractRawData(points);
    std::vector<double> raw_nrml = extractRawData(normals);

    std::cout << "Extracted data." << std::endl;

    ofile.add_properties_to_element("vertex", { "x", "y", "z" }, 
        tinyply::Type::FLOAT64, points.size(), reinterpret_cast<uint8_t*>(raw_pts.data()), tinyply::Type::INVALID, 0);

    ofile.add_properties_to_element("vertex", { "nx", "ny", "nz" },
    tinyply::Type::FLOAT64, normals.size(), reinterpret_cast<uint8_t*>(raw_nrml.data()), tinyply::Type::INVALID, 0);

    ofile.write(outstream, false);
}