#include <vector>
#include <algorithm>
#include <random>
#include <assert.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "trajectory.hpp"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Some constants
const double target_speed = 50.0 * 1609.0 / 3600.0;
const int num_candidate_trajectories = 100;
const FrenetPoint safety_zone_size {100, 3};

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
    // FrenetPoint target_end_pos(car.pos.s + target_speed, car.pos.d);
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