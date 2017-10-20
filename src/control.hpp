// Defines main control functions for the car

#include <vector>
#include <math.h>
#include "constants.hpp"


using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

// Follow current line
vector<vector<double>> follow_line(const double &car_s, const double &car_d, const double &car_x, const double &car_y,
	const double &car_yaw, const double &car_speed, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y,
	const vector<double> &maps_dx, const vector<double> &maps_dy,
    const vector<double> &previous_path_x, const vector<double> &previous_path_y, vector<vector<double>> sensor_fusion);

// Transform from global coordinates to local in the car referential pointing towards x axis
vector<double> global_to_local(const double &x, const double &y, const double &x_ref, const double &y_ref, const double &theta_ref);

// Transform from global to local entire vectors
void global_to_local(vector<double> &x, vector<double> &y, const double &x_ref, const double &y_ref, const double &theta_ref);

// Transform from local coordinates to global
vector<double> local_to_global(const double &x, const double &y, const double &x_ref, const double &y_ref, const double &theta_ref);

// Transform from local to global entire vectors
void local_to_global(vector<double> &x, vector<double> &y, const double &x_ref, const double &y_ref, const double &theta_ref);