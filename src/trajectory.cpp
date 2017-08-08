#include <iostream>
#include <vector>
#include <deque>
#include <algorithm>
#include <random>
#include <assert.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "trajectory.hpp"
#include "spline.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Some constants
const double max_speed = 45.0 * 1609.0 / 3600.0;
const double max_acceleration = 6.0;
const double time_step = 0.02;
const int num_candidate_trajectories = 100;
const FrenetPoint safety_zone_size {100, 3};
const double min_safe_s_distance = 40.0;

double max_waypoint_time(const Waypoints &waypoints) {
    if (waypoints.size() == 0) {
        return 0.0;
    }
    return waypoints.back().t;
}

double min_waypoint_time(const Waypoints &waypoints) {
    if (waypoints.size() == 0) {
        return 0.0;
    }
    return waypoints.front().t;
}

void get_target_lane_and_speed(FrenetPoint pos, const Cars &other_cars, double time_delta, int &target_lane, double &target_speed) {
    assert(time_delta > 0);
    vector<double> closest_ahead_distance = {10000, 10000, 10000};
    vector<Car> lane_cars_ahead(3);
    vector<bool> lane_free_ahead = {true, true, true};
    vector<bool> lane_free_for_change = {true, true, true};
    const int current_lane = lane_no(pos);

    for (Car car: other_cars) {
        int lane = lane_no(car.pos);
        FrenetPoint predicted_pos = car.predict_pos(time_delta);
        double distance = predicted_pos.s - pos.s;
        // Determine the closest car ahead
        if (distance > 0 && distance < closest_ahead_distance[lane]) {
            lane_cars_ahead[lane] = car;
            closest_ahead_distance[lane] = distance;
        }
        // Check if the lane is free. Consider cars that are both ahead and behind.
        if (distance > 0 && distance < min_safe_s_distance) {
            lane_free_ahead[lane] = false;
            lane_free_for_change[lane] = false;
        }
        else if (distance <= 0 && distance > -1.5 * min_safe_s_distance) {
            lane_free_for_change[lane] = false;
        }
    }
    // If current lane is free, keep driving it at max speed
    if (lane_free_ahead[current_lane]) {
        target_lane = current_lane;
        target_speed = max_speed;
        return;
    }
    // Check if any of the lanes are free for lane change
    for (int lane = 0; lane < 3; lane++) {
        if (lane_free_for_change[lane]) {
            target_lane = lane;
            target_speed = max_speed;
            return;
        }
    }
    // Keep driving the current lane, limit speed to same as the car ahead
    target_lane = current_lane;
    target_speed = min(max_speed, lane_cars_ahead[current_lane].vel.s);
}

void add_waypoint(Waypoints &waypoints, double time_step, const Cars &other_cars, double current_time) {
    assert(waypoints.size() > 0);
    const Waypoint &latest_wp = waypoints.back();
    double time_delta = latest_wp.t - current_time;
    assert(time_delta >= 0);
    double target_speed;
    int target_lane;
    get_target_lane_and_speed(latest_wp.pos, other_cars, time_delta, target_lane, target_speed);
    
    double speed_diff = target_speed - latest_wp.speed;
    double acc = speed_diff / time_step;
    // Clamp the acceleration between [-max, max]
    acc = min(max_acceleration, max(-max_acceleration, acc));
    double speed = latest_wp.speed + acc * time_step;
    double travel_distance = latest_wp.speed * time_step + 0.5 * acc * time_step * time_step;
    double d = lane_center(target_lane);
    FrenetPoint pos(latest_wp.pos.s + travel_distance, d);
    double t = latest_wp.t + time_step;
    assert(t > waypoints[0].t);
    waypoints.push_back(Waypoint(pos, speed, t));
}

void calculate_waypoint_splines(Waypoints waypoints, Transform2D sd_to_xy, tk::spline &x_spline, tk::spline &y_spline) {
    const int n = waypoints.size();
    assert(n >= 3);

    Coeffs t, x, y;
    for (Waypoint wp: waypoints) {
        t.push_back(wp.t);
        vector<double> xy = sd_to_xy(wp.pos.s, wp.pos.d);
        x.push_back(xy[0]);
        y.push_back(xy[1]);
    }   
    assert(t.size() == n);
    assert(t.size() == x.size());
    assert(t.size() == y.size());
    x_spline.set_points(t, x);
    y_spline.set_points(t, y);
}

void generate_car_path(Car ego_car, const Cars &other_cars, const int path_length, Transform2D sd_to_xy, const vector<double> prev_path_x, const vector<double> prev_path_y, vector<double> &path_x, vector<double> &path_y) {
    static Waypoints waypoints = {Waypoint(ego_car.pos, 0.0, 0.0)};
    const double waypoint_lookahead_time = 4.0;
    const double waypoint_lookbehind_time = 4.0;
    const double waypoint_time_step = 2.0;
    static double interpolation_time = 0.0;
    static tk::spline x_spline;
    static tk::spline y_spline;

    const int prev_path_length = prev_path_x.size();
    assert(prev_path_y.size() == prev_path_length);
    assert(prev_path_length <= path_length);
    double current_time = interpolation_time - prev_path_length * time_step;
    cout << "Current time = " << current_time << endl;
    // Check if we need to add a waypoint
    int new_point_count = path_length - prev_path_length;
    double max_interpolation_time = interpolation_time + new_point_count * time_step;
    bool recalculate_splines = false;
    cout << "Max interpolation time = " << max_interpolation_time << endl;
    while (max_waypoint_time(waypoints) - max_interpolation_time < waypoint_lookahead_time) {
        cout << "Adding waypoint" << endl;
        add_waypoint(waypoints, waypoint_time_step, other_cars, current_time);
        recalculate_splines = true;
    }
    // Discard too old waypoints
    while (max_interpolation_time - min_waypoint_time(waypoints) > waypoint_lookbehind_time) {
        waypoints.pop_front();
    }
    for (auto wp: waypoints) {
        cout << "Waypoint pos = " << wp.pos.s << ", " << wp.pos.d << " t = " << wp.t << endl;
    }
    if (recalculate_splines) {
        calculate_waypoint_splines(waypoints, sd_to_xy, x_spline, y_spline);
    }

    // Copy the old path to the beginning of the new path
    path_x = prev_path_x;
    path_y = prev_path_y;
    while (path_x.size() < path_length) {
        interpolation_time += time_step;
        double x = x_spline(interpolation_time);
        double y = y_spline(interpolation_time);
        path_x.push_back(x);
        path_y.push_back(y);
        cout << "Interpolating: t = " << interpolation_time << " xy = " << x << ", " << y << endl;
    }

    assert(path_x.size() == path_length && path_y.size() == path_length);
}
