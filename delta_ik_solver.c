#include <stdbool.h>
#include <math.h>

bool Delta_IK_Solver(double x, double y, double z, double R_base, double R_effector, double L_a, double L_b, double q[3]) {
    double R = R_base - R_effector;
    double phi[3] = {0.0, 2.0 * M_PI / 3.0, 4.0 * M_PI / 3.0}; // 0°, 120°, 240° in radians

    for (int i = 0; i < 3; ++i) {
        double cos_phi = cos(phi[i]);
        double sin_phi = sin(phi[i]);

        double x_i = x * cos_phi - y * sin_phi;
        double z_i = z;

        double x_i_minus_R = x_i - R;
        double A = (x_i_minus_R * x_i_minus_R + z_i * z_i + L_a * L_a - L_b * L_b) / (2.0 * L_a);
        
        double C = -x_i_minus_R - A;
        double B = 2.0 * z_i;
        double D = x_i_minus_R - A;

        double t_solutions[2];
        int num_solutions = 0;

        // Solve quadratic equation C*t² + B*t + D = 0
        if (C != 0.0) {
            double discriminant = B * B - 4.0 * C * D;
            if (discriminant < 0.0) {
                return false; // No real solutions
            }
            double sqrt_d = sqrt(discriminant);
            t_solutions[0] = (-B + sqrt_d) / (2.0 * C);
            t_solutions[1] = (-B - sqrt_d) / (2.0 * C);
            num_solutions = 2;
        } else {
            // Linear equation B*t + D = 0
            if (B == 0.0) {
                if (D == 0.0) {
                    return false; // Infinite solutions (unlikely in practice)
                } else {
                    return false; // No solution
                }
            } else {
                t_solutions[0] = -D / B;
                num_solutions = 1;
            }
        }

        // Check solutions for valid joint angle
        bool found_valid = false;
        for (int j = 0; j < num_solutions; ++j) {
            double t = t_solutions[j];
            double q_i = 2.0 * atan(t);

            if (q_i >= -M_PI_2 && q_i <= M_PI_2) {
                q[i] = q_i;
                found_valid = true;
                break;
            }
        }

        if (!found_valid) {
            return false;
        }
    }

    return true;
}
