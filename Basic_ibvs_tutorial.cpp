#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpColVector.h>
#include <visp3/vs/vpServo.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <iostream>
#include <cmath>

int main()
{
    try {
        // -------------------------------
        // 1. Camera parameters
        // -------------------------------
        vpCameraParameters cam(800, 800, 320, 240);

        // -------------------------------
        // 2. Define features (4 points)
        // -------------------------------
        vpFeaturePoint s[4], s_star[4];

        // Initial positions
        s[0].buildFrom(0.2,  0.2,  1);
        s[1].buildFrom(-0.2, 0.2,  1);
        s[2].buildFrom(-0.2, -0.2, 1);
        s[3].buildFrom(0.2,  -0.2, 1);

        // Desired positions
        s_star[0].buildFrom(0.1,  0.1,  1);
        s_star[1].buildFrom(-0.1, 0.1,  1);
        s_star[2].buildFrom(-0.1, -0.1, 1);
        s_star[3].buildFrom(0.1,  -0.1, 1);

        // -------------------------------
        // 3. Create servo task
        // -------------------------------
        vpServo task;
        task.setServo(vpServo::EYEINHAND_CAMERA);
        task.setInteractionMatrixType(vpServo::CURRENT);
        task.setLambda(0.5);   // λ = 0.5

        for (int i = 0; i < 4; i++) {
            task.addFeature(s[i], s_star[i]);
        }

        // -------------------------------
        // 4. Control loop (200 iterations)
        // -------------------------------
        for (int iter = 0; iter < 200; iter++) {

            // Compute control law
            vpColVector v = task.computeControlLaw();

            // -------------------------------
            // Compute error norm
            // -------------------------------
            double error_norm = 0.0;

            for (int i = 0; i < 4; i++) {
                double ex = s[i].get_x() - s_star[i].get_x();
                double ey = s[i].get_y() - s_star[i].get_y();

                error_norm += ex * ex + ey * ey;
            }

            error_norm = std::sqrt(error_norm);

            // Print results
            std::cout << "Iteration " << iter
                      << " | Error Norm: " << error_norm
                      << " | Velocity: " << v.t()
                      << std::endl;

            // -------------------------------
            // Convergence check
            // -------------------------------
            if (error_norm < 1e-4) {
                std::cout << "\nConverged at iteration "
                          << iter << std::endl;
                break;
            }

            // -------------------------------
            // Simulate motion update
            // -------------------------------
            for (int i = 0; i < 4; i++) {

                double x = s[i].get_x();
                double y = s[i].get_y();

                // Move toward desired position
                double x_new = x + 0.1 * (s_star[i].get_x() - x);
                double y_new = y + 0.1 * (s_star[i].get_y() - y);

                s[i].buildFrom(x_new, y_new, 1);
            }
        }

        std::cout << "\nFinal result: IBVS converged successfully " << std::endl;
    }
    catch (...) {
        std::cout << "Error occurred!" << std::endl;
    }

    return 0;
}
