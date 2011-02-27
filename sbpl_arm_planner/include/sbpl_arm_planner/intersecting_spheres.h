/* \author Gokul Subramanian */

#include <iostream>
#include <cstdlib>
#include <math.h>
#include <vector>

using namespace std;

namespace sbpl_arm_planner{

/* \brief Distance between two points (expects the cartesian coordinates) */
double distance_between(const double pt1[3], const double pt2[3]);

/* \brief Cross product of 2 vectors (3D) */
void cross_product(double (&result)[3], double vect1[3], double vect2[3]);

/* \brief Function to return a list of points at which the two sphere with the given
centers (x, y, z) and given radii r, intersect. */
std::vector<std::vector<double> > points_of_intersection(double x1, double y1, double z1, double r1, double x2, double y2, double z2, double r2, double angle_res,  std::vector<std::vector<double> > &invalid_points);

}
