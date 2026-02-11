#ifndef ROBOTICS_CORE_DIFFERENTIAL_DRIVE_HPP
#define ROBOTICS_CORE_DIFFERENTIAL_DRIVE_HPP

#include <vector>
#include <algorithm>
#include <cmath>

namespace robotics_core {

struct DifferentialParams {
    double track_width; // W: Distance between left and right wheels
    double wheel_radius; // R: Radius of the wheels
    double max_speed;    // Maximum wheel speed in rad/s
    std::vector<int> motor_directions;
};

class DifferentialKinematics {
public:
    explicit DifferentialKinematics(DifferentialParams params) : p_(params) {}

    // Input: Linear X (m/s) and Angular Z (rad/s)
    // Output: Wheel velocities in rad/s [L, R]
    std::vector<double> calculateWheelSpeeds(double vx, double wz) {
        double k = (p_.track_width) / 2.0;

        std::vector<double> speeds = {
            (vx - k * wz) / p_.wheel_radius * p_.motor_directions[0], // left 
            (vx + k * wz) / p_.wheel_radius * p_.motor_directions[1], // Right
        };

        return normalizeWheelSpeeds(speeds);
    }

    // Normalize Wheel Speeds
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
    DifferentialParams p_;
};
} // namespace robotics_core
#endif