// Defines main control functions for the car

#include "control.hpp"
#include <vector>
#include "spline.h"
#include "math.h"


using namespace std;


// For keeping track externally of lane change
bool g_changing_lane = false;
int g_target_lane = 0;

// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint = (closestWaypoint + 1) % maps_x.size(); // to ensure we go back to 0 if at the last item
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}
	if (prev_wp == -1) // in case we have a negative s
	{
		prev_wp = maps_s.size() - 1;
	}

	int wp2 = (prev_wp+1)%maps_x.size(); // in case we are at the last item

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}


// Follow current line
vector<vector<double>> follow_line(const double &car_s, const double &car_d, const double &car_x, const double &car_y,
	const double &car_yaw, const double &car_speed, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y,
	const vector<double> &maps_dx, const vector<double> &maps_dy,
    const vector<double> &previous_path_x, const vector<double> &previous_path_y, vector<vector<double>> sensor_fusion)
{
	// Define resulting vectors of x, y coordinates
	vector<double> pos_x, pos_y;

	int keep_points = 10;   		   // Number of points from previous trajectory we will keep
	int spline_prev_trajectory = 3;    // Number of points from previous trajectory we use for spline
	int previous_path_size = previous_path_x.size();
	int prev_points_used = min(previous_path_size, keep_points);

	// Define target lane if it has not been initialized yet and save it in global variable
	if (!g_target_lane)
	{
		g_target_lane = int(round((car_d + constants::LANE_WIDTH/2) / constants::LANE_WIDTH));	  // current lane
	}
	double target_d = constants::LANE_WIDTH * (g_target_lane - 1. / 2.);
	double target_horizon = 15.;	 // how far ahead we look to define our trajectory

	// See if we will be too close to other cars in 3 seconds based on current velocities
	bool too_close = false;
	bool try_change_lane = false;
	double time_horizon = 1.;
	double critical_distance = 6.;		// at this distance, we slow down
	double prepare_lane_change = 30.;	// we anticipate by trying to change lane early
	double my_future_car_s = car_s + car_speed * 1609 / 3600 * time_horizon;
	
	// scan all the cars from sensor fusion
	for (auto &other_car : sensor_fusion)
	{
		double other_car_lane = (other_car[6] + constants::LANE_WIDTH/2) / constants::LANE_WIDTH;
		if (abs(other_car_lane - g_target_lane) < 0.7)  // we are in the same lane or getting close
		{
			double other_car_s = other_car[5];
			double other_car_speed = sqrt(other_car[3] * other_car[3] + other_car[4] * other_car[4]);
			double other_future_car_s = other_car_s + other_car_speed * 1609 / 3600 * time_horizon;
			if ((((other_future_car_s - critical_distance) < my_future_car_s) || ((other_car_s - critical_distance) < car_s))
				&& (car_s < other_car_s))	// we are either already too close or about to be too close
			{
				too_close = true;
				try_change_lane = true;
				break;
			}
			if ((((other_future_car_s - prepare_lane_change) < my_future_car_s) || ((other_car_s - prepare_lane_change) < car_s))
				&& (car_s < other_car_s))	// we are either already too close or about to be too close
			{
				try_change_lane = true;    // we try to change lane before we have to slow down
			}
		}
	}

	// Check if we want to change lane as long as we are not already changing lane
	if (try_change_lane && !g_changing_lane)
	{
		double safety_lane_change_distance_back = 8.;
		double safety_lane_change_distance_front = 15.;
		bool left_lane_free = true;
		bool right_lane_free = true;
		if (g_target_lane == 1) // we are on the left lane
		{
			left_lane_free = false;
		}
		if (g_target_lane == 3) // we are on the right lane
		{
			right_lane_free = false;
		}
		
		// scan all the cars from sensor fusion
		for (auto &other_car : sensor_fusion)
		{
			double other_car_s = other_car[5];
			double other_car_speed = sqrt(other_car[3] * other_car[3] + other_car[4] * other_car[4]);
			double other_future_car_s = other_car_s + other_car_speed * 1609 / 3600 * time_horizon;
			
			if ((((other_car_s - car_s) > - safety_lane_change_distance_back)   // We check that we are currently between safety margins of other cars
			   && ((other_car_s - car_s) < safety_lane_change_distance_front))
			   || (((other_future_car_s - my_future_car_s) > - safety_lane_change_distance_back)  // and ensure it is also valid in the future
			   && ((other_future_car_s - my_future_car_s) < safety_lane_change_distance_front))
			   // and ensure the car didn't go through another car (and our safety margin) during our prediction time
			   || ((other_car_s < car_s - safety_lane_change_distance_back) && (other_future_car_s > my_future_car_s + safety_lane_change_distance_front))
			   || ((other_future_car_s < my_future_car_s - safety_lane_change_distance_back) && (other_car_s > car_s + safety_lane_change_distance_front)))
			{
				double other_car_lane = (other_car[6] + constants::LANE_WIDTH/2) / constants::LANE_WIDTH;
				if (((other_car_lane - g_target_lane) < -0.5) && ((other_car_lane - g_target_lane) > -1.5))  // other car is on our left
				{
					left_lane_free = false;
				}
				if (((other_car_lane - g_target_lane) > 0.5) && ((other_car_lane - g_target_lane) < 1.5))  // other car is on our right
				{
					right_lane_free = false;
				}
			}
		}

		// Change lane if one is free
		if (left_lane_free)
		{
			g_changing_lane = true;
			g_target_lane -= 1;
		}
		else if (right_lane_free)
		{
			g_changing_lane = true;
			g_target_lane += 1;
		}
	}

	if (g_changing_lane)
	{
		target_horizon = 35.;    // This ensure we don't change lane too fast
		target_d = constants::LANE_WIDTH * (g_target_lane - 1. / 2.);

		// Check if we are done changing lane
		if (abs((car_d + constants::LANE_WIDTH/2) / constants::LANE_WIDTH - g_target_lane) < 0.25)
		{
			g_changing_lane = false; // we have finished our change of lane
		}
	}

	// Define our target speed
	double target_speed = car_speed * 1609 / 3600;
	if (too_close)
	{
		target_speed -= constants::TARGET_ACC * constants::REFRESH_TIME * keep_points;
	}
	else
	{
		target_speed += constants::TARGET_ACC * constants::REFRESH_TIME * keep_points;
		target_speed = min(target_speed, constants::TARGET_SPEED * 1609 / 3600);
	}
	
	// Create a spline based on 3 points: current position and 2 points front and back of the car
	tk::spline s_xy;	
	
	if (previous_path_size > spline_prev_trajectory)
	{
		std::vector<double> s_X(spline_prev_trajectory + 2), s_Y(spline_prev_trajectory + 2);
		
		// add last trajectory points
		for (int i = 0; i<spline_prev_trajectory; ++i)
		{
			s_X[i] = previous_path_x[prev_points_used-spline_prev_trajectory+i];
			s_Y[i] = previous_path_y[prev_points_used-spline_prev_trajectory+i];
		}
		
		// add points based on last point from target trajectory + a small distance (target_horizon)
		double last_x = s_X[spline_prev_trajectory - 1];
		double last_y = s_Y[spline_prev_trajectory - 1];
		double last_x2 = s_X[spline_prev_trajectory - 2];
		double last_y2 = s_Y[spline_prev_trajectory - 2];
		double last_theta = atan2(last_y-last_y2, last_x-last_x2);
		auto frenet = getFrenet(last_x, last_y, last_theta, maps_x, maps_y);	
		auto next_z = getXY(frenet[0] + target_horizon, target_d, maps_s, maps_x, maps_y);
		s_X[spline_prev_trajectory] = next_z[0];
		s_Y[spline_prev_trajectory] = next_z[1];
		next_z = getXY(frenet[0] + 2 * target_horizon, target_d, maps_s, maps_x, maps_y);
		s_X[spline_prev_trajectory + 1] = next_z[0];
		s_Y[spline_prev_trajectory + 1] = next_z[1];

		// Convert all points to local coordinates
		global_to_local(s_X, s_Y, car_x, car_y, car_yaw);

		// create the spline
		s_xy.set_points(s_X, s_Y);
	}
	else
	// We don't have enough points so we will use last waypoint, current position and next waypoint
	{
		std::vector<double> s_X(3), s_Y(3);
		auto prev_z = getXY(car_s - target_horizon, target_d, maps_s, maps_x, maps_y);
		auto next_z = getXY(car_s + target_horizon, target_d, maps_s, maps_x, maps_y);
		s_X[0] = prev_z[0];
		s_Y[0] = prev_z[1];
		s_X[1] = car_x;
		s_Y[1] = car_y;
		s_X[2] = next_z[0];
		s_Y[2] = next_z[1];

		// Convert all points to local coordinates
		global_to_local(s_X, s_Y, car_x, car_y, car_yaw);
		
		// create the spline
		s_xy.set_points(s_X, s_Y);
	}
	
	// Keep part of the previous trajectory if possible
	for (int i=0; i < prev_points_used; ++i)
	{
		pos_x.push_back(previous_path_x[i]);
		pos_y.push_back(previous_path_y[i]);
	}

	// Get last point we took from previous trajectory, use current car position if not available
	double last_x = car_x;
	double last_y = car_y;
	if (prev_points_used)
	{
		last_x = pos_x.back();
		last_y = pos_y.back();
	}
	auto last_z_local = global_to_local(last_x, last_y, car_x, car_y, car_yaw);
	double last_x_local = last_z_local[0];
	double last_y_local = last_z_local[1];

	// Add new points based on target speed
	double distance_inc = target_speed * constants::REFRESH_TIME; // target distance in m
	for (int i = pos_x.size(); i<50; ++i)  // have a total of 50 elements, equivalent to 1 sec
	{		
		// Look for (x, y) approximately at distance_inc
		double next_x_local = last_x_local + distance_inc;
		double next_y_local = s_xy(next_x_local);
		double d = distance(last_x_local, last_y_local, next_x_local, next_y_local);
		
		// Readjust (x, y) based on calculated distance
		next_x_local = last_x_local + (next_x_local - last_x_local) * distance_inc / d;
		next_y_local = s_xy(next_x_local);

		// Add calculated positions to trajectory
		auto next_z_global = local_to_global(next_x_local, next_y_local, car_x, car_y, car_yaw);
		pos_x.push_back(next_z_global[0]);
		pos_y.push_back(next_z_global[1]);		
		
		// Update last points of current trajectory
		last_x_local = next_x_local;
		last_y_local = next_y_local;
	}

	return {pos_x, pos_y};	
}

// Transform from global coordinates to local in the car referential pointing towards x axis
vector<double> global_to_local(const double &x, const double &y, const double &x_ref, const double &y_ref, const double &theta_ref)
{
	// Local coordinates
	double xl, yl, xc, yc;

	xc = x - x_ref;
	yc = y - y_ref;

	xl = xc * cos(deg2rad(theta_ref)) + yc * sin(deg2rad(theta_ref));
	yl = yc * cos(deg2rad(theta_ref)) - xc * sin(deg2rad(theta_ref));

	return {xl, yl};
}

// Transform from global to local entire vectors
void global_to_local(vector<double> &x, vector<double> &y, const double &x_ref, const double &y_ref, const double &theta_ref)
{
	for (int i=0; i < x.size(); ++i)
	{
		auto z = global_to_local(x[i], y[i], x_ref, y_ref, theta_ref);
		x[i] = z[0];
		y[i] = z[1];
	}
}

// Transform from local coordinates to global
vector<double> local_to_global(const double &x, const double &y, const double &x_ref, const double &y_ref, const double &theta_ref)
{
	// Local coordinates
	double xg, yg;

	xg = x * cos(deg2rad(theta_ref)) - y * sin(deg2rad(theta_ref));
	yg = y * cos(deg2rad(theta_ref)) + x * sin(deg2rad(theta_ref));

	xg += x_ref;
	yg += y_ref;

	return {xg, yg};
}

// Transform from local to global entire vectors
void local_to_global(vector<double> &x, vector<double> &y, const double &x_ref, const double &y_ref, const double &theta_ref)
{
	for (int i=0; i < x.size(); ++i)
	{
		auto z = local_to_global(x[i], y[i], x_ref, y_ref, theta_ref);
		x[i] = z[0];
		y[i] = z[1];
	}
}