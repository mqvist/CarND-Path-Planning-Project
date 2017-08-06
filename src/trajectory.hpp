enum class Behavior {
    keep_lane, change_lane_left, change_lane_right
};

struct FrenetPoint {
    double s, d;

    FrenetPoint(double s, double d) : s(s), d(d) {}
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

typedef std::vector<FrenetPoint> FrenetPoints;
typedef std::vector<Car> Cars;
typedef std::vector<double> Coeffs;

inline int lane(FrenetPoint p) { return p.d / 4; }

FrenetPoints generate_car_trajectory(EgoCar car, Behavior behavior, const Cars other_cars, double time_step);

