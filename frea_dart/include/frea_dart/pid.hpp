#ifndef __PID_HPP__
#define __PID_HPP__

class Pid {
public:
    /**
     * @brief Constructor
     */
    Pid();

    /**
     * @brief Destructor
     */
    ~Pid();

    /**
     * @brief Sets the pid gains
     */
    void setGains(double kp, double ki, double kd);

    /**
     * @brief Resets the pid controller
     */
    void reset();

    /**
     * @brief Updates the pid controller
     */
    double update(double process_value, double target, double dt);

private:
    double kp_;

    double ki_;

    double kd_;

    double int_err_;

    double prev_err_;
};

#endif // __PID_HPP__
