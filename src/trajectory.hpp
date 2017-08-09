using std::vector;
using std::deque;

struct FrenetPoint {
    double s, d;

    FrenetPoint() : s(0.0), d(0.0) {}
    FrenetPoint(double s, double d) : s(s), d(d) {}
};

struct Waypoint {
    FrenetPoint pos;
    double speed;
    double t;

    Waypoint(FrenetPoint pos, double speed, double t) : pos(pos), speed(speed), t(t) {}
};

struct Car {
    FrenetPoint pos;
    FrenetPoint vel;

    Car() {}
    Car(FrenetPoint pos, FrenetPoint vel) : pos(pos), vel(vel) {}

    FrenetPoint predict_pos(double time_step) {
        return FrenetPoint(pos.s + vel.s * time_step, pos.d + vel.d * time_step);
    }
};

typedef vector<FrenetPoint> FrenetPoints;
typedef vector<Car> Cars;
typedef vector<double> Coeffs;
typedef deque<Waypoint> Waypoints;
typedef std::function<vector<double>(double, double)> Transform2D;
inline int lane_no(FrenetPoint p) { return p.d / 4; }
inline double lane_center(int lane_no) { return 2.0 + lane_no * 4.0; }

// FrenetPoints generate_car_trajectory(EgoCar car, Behavior behavior, const Cars other_cars, double time_step);
void generate_car_path(Car ego_car, const Cars &other_cars, int path_length, Transform2D sd_to_xy, const vector<double> prev_path_x, const vector<double> prev_path_y, vector<double> &path_x, vector<double> &path_y);
                    

