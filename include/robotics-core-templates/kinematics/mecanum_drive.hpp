#ifndef ROBOTICS_CORE_MECHNUM_DRIVE_HPP
#define ROBOTICS_CORE_MECHNUM_DRIVE_HPP

#include <vector>
#include <algorithm>
#include <cmath>

namespace robotics_core {

struct MecanumParams {
    double wheel_base;  // L: Distance between front and back axle
    double track_width; // W: Distance between left and right wheels
    double wheel_radius; // R: Radius of the wheels
    double max_speed;    // Maximum wheel speed in rad/s
    std::vector<int> directions;
};

class MecanumKinematics {
public:
    explicit MecanumKinematics(MecanumParams params) : p_(params) {}

    // Input: Linear X, Y (m/s) and Angular Z (rad/s)
    // Output: Wheel velocities in rad/s [FL, FR, BL, BR]
    std::vector<double> calculateWheelSpeeds(double vx, double vy, double wz) {
        double k = (p_.wheel_base + p_.track_width) / 2.0;

        std::vector<double> speeds = {
            (vx - vy - k * wz) / p_.wheel_radius * p_.directions[0], //Front left 
            (vx + vy + k * wz) / p_.wheel_radius * p_.directions[1], //Front Right
            (vx + vy - k * wz) / p_.wheel_radius * p_.directions[2], // Back Left 
            (vx - vy + k * wz) / p_.wheel_radius * p_.directions[3]  // Back Right 
        };
        return normalizeWheelSpeeds(speeds);
    }

    //Normalise Wheel Speeds
    std::vector<double>normalizeWheelSpeeds(const std::vector<double>& speeds) {
        std::vector<double> normalized_speeds=speeds;
        auto max_it = std::max_element(speeds.begin(), speeds.end(), [](double a, double b) {
            return std::abs(a) < std::abs(b);
        });

        double max_abs_speed = std::abs(*max_it);

        // Avoid Division by Zero and scale if exceed Max Speed
        if (max_abs_speed > p_.max_speed && max_abs_speed > 1e-6) { 
            double scale = p_.max_speed / max_abs_speed;
            for (double &speed : normalized_speeds) {
                speed *= scale;
            }
        }

        return normalized_speeds;
    }




private:
    MecanumParams p_;
};
} // namespace robotics_core
#endif