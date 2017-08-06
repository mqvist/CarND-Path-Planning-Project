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
const double max_acceleration = 8.0;
const double time_step = 0.02;
const int num_candidate_trajectories = 100;
const FrenetPoint safety_zone_size {100, 3};
const double min_safe_s_distance = 20.0;

// FrenetPoints sample_positions(FrenetPoint origin, double s_std_dev, int num_samples) {
//     default_random_engine generator;
//     normal_distribution<double> distribution(origin.s, s_std_dev);

//     FrenetPoints samples;
//     for (int i = 0; i < num_samples; ++i) {
//         double s = distribution(generator);
//         samples.push_back({s, origin.d});
//     }
//     return samples;
// }

FrenetPoints interpolate_position(FrenetPoint start_pos, FrenetPoint end_pos, int num_points) {
    assert(end_pos.s > start_pos.s);
    FrenetPoints points;
    double delta_s = (end_pos.s - start_pos.s) / num_points;
    double delta_d = (end_pos.d - start_pos.d) / num_points;
    for (int i = 1; i <= num_points; ++i) {
        double s = start_pos.s + i * delta_s;
        double d = start_pos.d + i * delta_d;
        points.push_back({s, d});
    }
    assert(points.size() == num_points);
    assert(points[num_points - 1].s == end_pos.s && points[num_points - 1].d == end_pos.d);
    return points;
}

Cars predict_car_positions(Cars cars, double time_step) {
    Cars predicted_cars;

    for (Car car: cars) {
        predicted_cars.push_back(car.predict(time_step));
    }

    return predicted_cars;
}

// double safe_zone_penalty(FrenetPoint car_position, FrenetPoints other_car_positions) {
//     double penalty = 0.0;
//     double zone_s_start = car_position.s;
//     double zone_s_end = car_position.s + safe_zone_size.s;

//     for(FrenetPoint other_pos: other_car_positions) {
//         // Fast check if the safe zone's can overlap
//         if (other_pos.s >= zone_s_start && other_pos.s <= zone_end.end)
//     }
//     return penalty;
// }

double same_lane_penalty(FrenetPoint pos, const Cars other_cars) {
    double penalty = 0.0;
    double s_start = pos.s;
    double s_end = pos.s + safety_zone_size.s;

    for(auto other_car: other_cars) {
        // Ignore cars that are not on the same lane as us
        if (lane(other_car.pos) != lane(pos))
            continue;
        // Ignore cars that do not fall on our safety zone
        if (other_car.pos.s < s_start || other_car.pos.s > s_end) 
            continue;
        // Penalize by the distance the car intrudes into the zone
        double intrusion = s_end - other_car.pos.s;
        assert(intrusion >= 0);
        penalty += intrusion * intrusion;
    }

    return penalty;
}

vector<double> calculate_position_costs(const FrenetPoints positions, FrenetPoint target_pos, const Cars car_predictions) {
    vector<double> costs;

    for(auto const pos: positions) {
        assert(pos.s <= target_pos.s);
        // Prefer getting closer to the target
        double cost = target_pos.s - pos.s;
        // Penalize getting too close to cars on the same lane
        cost += same_lane_penalty(pos, car_predictions);
    }

    assert(costs.size() == positions.size());
    return costs;
}

Coeffs JMT(Coeffs start, Coeffs end, double T) {
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    Eigen::Matrix3f A;
    A << T3, T4, T5,
         3 * T2, 4 * T3, 5 * T4,
         6 * T, 12 * T2, 20 * T3;

    Eigen::Vector3f b;
    b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T2),
         end[1] - (start[1] + start[2] * T),
         end[2] - start[2];

    Eigen::Vector3f x = A.inverse() * b;

    return {start[0], start[1], 0.5 * start[2], x[0], x[1], x[2]};
}

double evaluate_poly(Coeffs coeffs, double at) {
    double value = 0.0;
    double term = 1.0;

    for (double coeff: coeffs) {
        value += coeff * term;
        term *= at;
    }
    return value;
}

FrenetPoints calculate_jerk_minimizing_trajectory(EgoCar car_now, EgoCar car_then, double time_step) {
    Coeffs s_start = {car_now.pos.s, car_now.vel.s, car_now.acc.s};
    Coeffs d_start = {car_now.pos.d, car_now.vel.d, car_now.acc.d};
    Coeffs s_end = {car_then.pos.s, car_then.vel.s, car_then.acc.s};
    Coeffs d_end = {car_then.pos.d, car_then.vel.d, car_then.acc.d};

    Coeffs s_coeffs = JMT(s_start, s_end, time_step);
    Coeffs d_coeffs = JMT(d_start, d_end, time_step);

    double time_delta = 0.02;
    int num_points = time_step / time_delta;
    FrenetPoints trajectory;
    for (int i = 0; i < num_points; ++i) {
        double time = (i + 1) * time_delta;
        double s = evaluate_poly(s_coeffs, time);
        double d = evaluate_poly(d_coeffs, time);
        trajectory.push_back(FrenetPoint(s, d));
    }

    return trajectory;
}

FrenetPoints generate_car_trajectory(EgoCar car, Behavior behavior, const Cars other_cars, double time_step) {

    assert(behavior == Behavior::keep_lane);

    // Cars car_predictions = predict_car_positions(other_cars, time_step);
    // // Assuming we drive at the target speed for one second
    // FrenetPoint target_end_pos(car.pos.s + max_speed, car.pos.d);
    // // Sample different end positions around the target
    // FrenetPoints candidate_end_positions = interpolate_position(car.pos, target_end_pos, 10);
    // // Calculate candidate end positions costs
    // vector<double> candidate_costs = calculate_position_costs(candidate_end_positions, target_end_pos, car_predictions);
    // // Pick the lowest cost end position
    // auto best = min_element(candidate_costs.begin(), candidate_costs.end());
    // int best_index = best - candidate_costs.begin();
    // FrenetPoint best_pos = candidate_end_positions[best_index];

    FrenetPoint pos_then(car.pos.s + time_step * 10, car.pos.d);
    FrenetPoint vel_then(20, 0);
    FrenetPoint acc_then(0, 0);
    EgoCar car_then(pos_then, vel_then, acc_then);

    // Generate Jerk minimizing trajectory to the best position
    return calculate_jerk_minimizing_trajectory(car, car_then, time_step);
}

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

bool get_closest_car_ahead(const Cars &cars, FrenetPoint pos, double time_delta, Car &closest_car) {
    double closest_distance = 10000;
    bool closest_found = false;
    for (Car car: cars) {
        // Ignore cars that are not on the same lane as us
        if (lane(car.pos) != lane(pos))
            continue;
        Car predicted_car = car.predict(time_delta);
        double distance = predicted_car.pos.s - pos.s;
        cout << "Car distance = " << distance << endl;
        if (distance >= 0.0 && distance < closest_distance) {
            closest_car = predicted_car;
            closest_distance = distance;
            closest_found = true;
        }
    }
    return closest_found;
}

void add_waypoint(Waypoints &waypoints, double time_step, const Cars &other_cars, double current_time) {
    assert(waypoints.size() > 0);
    const Waypoint &latest_wp = waypoints.back();
    double target_speed = max_speed;
    double time_delta = latest_wp.t - current_time;
    assert(time_delta >= 0);
    Car car_ahead;
    if (get_closest_car_ahead(other_cars, latest_wp.pos, time_delta, car_ahead)) {
        double distance = car_ahead.pos.s - latest_wp.pos.s;
        if (distance < min_safe_s_distance) {
            target_speed = car_ahead.vel.s;
        }
    }

    double speed_diff = target_speed - latest_wp.speed;
    double acc = speed_diff / time_step;
    // Clamp the acceleration between [-max, max]
    acc = min(max_acceleration, max(-max_acceleration, acc));
    double speed = latest_wp.speed + acc * time_step;
    double travel_distance = latest_wp.speed * time_step + 0.5 * acc * time_step * time_step;
    FrenetPoint pos(latest_wp.pos.s + travel_distance, latest_wp.pos.d);
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

void generate_car_path(EgoCar ego_car, const Cars &other_cars, const int path_length, Transform2D sd_to_xy, const vector<double> prev_path_x, const vector<double> prev_path_y, vector<double> &path_x, vector<double> &path_y) {
    static Waypoints waypoints = {Waypoint(ego_car.pos, 0.0, 0.0)};
    const double waypoint_lookahead_time = 3.0;
    const double waypoint_lookbehind_time = 3.0;
    const double waypoint_time_step = 1.0;
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
