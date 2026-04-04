#include <visp3/core/vpColVector.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>

// Function to generate random number in range
double random_double(double min, double max) {
    return min + (max - min) * ((double)rand() / RAND_MAX);
}

int main()
{
    srand((unsigned int)time(0));

    // -------------------------------
    // Desired end-effector position (target)
    // -------------------------------
    vpColVector desired(3);
    desired[0] = 0.0;   // x (meters)
    desired[1] = 0.0;   // y
    desired[2] = 0.5;   // z

    std::vector<double> errors;

    // -------------------------------
    // 10 randomized initial poses
    // -------------------------------
    for (int i = 0; i < 10; i++) {

        vpColVector current(3);

        // Random initial pose (simulate robot inaccuracy)
        current[0] = random_double(-0.1, 0.1);   
        current[1] = random_double(-0.1, 0.1);
        current[2] = random_double(0.3, 0.7);    // depth variation

        // -------------------------------
        // Compute Euclidean error
        // -------------------------------
        double error = std::sqrt(
            std::pow(current[0] - desired[0], 2) +
            std::pow(current[1] - desired[1], 2) +
            std::pow(current[2] - desired[2], 2)
        );

        // Convert to mm
        error = error * 1000.0;

        errors.push_back(error);

        std::cout << "Test " << i
            << " | Initial Pose: ("
            << current[0] << ", "
            << current[1] << ", "
            << current[2] << ")"
            << " | Error (mm): " << error
            << std::endl;
    }

    // -------------------------------
    // Compute mean
    // -------------------------------
    double sum = 0.0;
    for (double e : errors)
        sum += e;

    double mean = sum / errors.size();

    // -------------------------------
    // Compute standard deviation
    // -------------------------------
    double variance = 0.0;
    for (double e : errors)
        variance += std::pow(e - mean, 2);

    variance /= errors.size();
    double stddev = std::sqrt(variance);

    // -------------------------------
    // Final result
    // -------------------------------
    std::cout << "\nMean Error: " << mean << " mm" << std::endl;
    std::cout << "Std Dev: ±" << stddev << " mm" << std::endl;

    return 0;
}
