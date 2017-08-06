using std::vector;
using std::deque;

enum class Behavior {
    keep_lane, change_lane_left, change_lane_right
};

struct FrenetPoint {
    double s, d;

    FrenetPoint(double s, double d) : s(s), d(d) {}
};

struct Waypoint {
    FrenetPoint pos;
    double t;

    Waypoint(FrenetPoint pos, double t) : pos(pos), t(t) {}
};

struct Car {
    FrenetPoint pos;
    FrenetPoint vel;

    Car(FrenetPoint pos, FrenetPoint vel) : pos(pos), vel(vel) {}

    Car predict(double time_step) {
        return Car(FrenetPoint(pos.s + vel.s * time_step, pos.d + vel.d * time_step), vel);
    }
};

struct EgoCar : public Car {
    FrenetPoint acc;

    EgoCar(FrenetPoint pos, FrenetPoint vel, FrenetPoint acc) : Car(pos, vel), acc(acc) {}
};

typedef vector<FrenetPoint> FrenetPoints;
typedef vector<Car> Cars;
typedef vector<double> Coeffs;
typedef deque<Waypoint> Waypoints;
typedef std::function<vector<double>(double, double)> Transform2D;
inline int lane(FrenetPoint p) { return p.d / 4; }

// FrenetPoints generate_car_trajectory(EgoCar car, Behavior behavior, const Cars other_cars, double time_step);
void generate_car_path(EgoCar ego_car, int path_length, Transform2D sd_to_xy, const vector<double> prev_path_x, const vector<double> prev_path_y, vector<double> &path_x, vector<double> &path_y);
                    

