#include "frea_dart/pid.hpp"

Pid::Pid() {
    setGains(0, 0, 0);
    reset();
}

Pid::~Pid() {
    // dtor
}

void Pid::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void Pid::reset() {
    int_err_ = 0;
    prev_err_ = 0;
}

double Pid::update(double process_value, double target, double dt) {
    double err = target - process_value;
    int_err_ += err * dt;

    double ret = 0;
    if(dt > 0) {
        double d = (err - prev_err_) / dt;
        ret = err * kp_ + int_err_ * ki_ + kd_ * d;
    }
    prev_err_ = err;

    return ret;
}
