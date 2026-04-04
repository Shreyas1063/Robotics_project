#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/visual_features/vpFeaturePoint.h>

#include <iostream>
#include <vector>

int main()
{
    try {
        // -------------------------------
        // 1. Camera parameters
        // -------------------------------
        vpCameraParameters cam(800, 800, 320, 240);

        // -------------------------------
        // 2. Load image
        // -------------------------------
        vpImage<unsigned char> I;

        //  Putting image in project folder
        vpImageIo::read(I, "C:\\Users\\Hp\\source\\repos\\Visptest\\x64\\Release\\marker.jpg");

        // -------------------------------
        // 3. AprilTag detector
        // -------------------------------
        vpDetectorAprilTag detector(vpDetectorAprilTag::TAG_36h11);

        detector.setAprilTagQuadDecimate(1.0);

        // -------------------------------
        // 4. Detect marker
        // -------------------------------
        bool detected = detector.detect(I);

        if (!detected) {
            std::cout << "Marker not detected!" << std::endl;
            return 0;
        }

        std::cout << "Marker detected " << std::endl;

        // -------------------------------
        // 5. Get corner points
        // -------------------------------
        std::vector<std::vector<vpImagePoint>> polygons = detector.getPolygon();

        if (polygons.empty()) {
            std::cout << "No corners found!" << std::endl;
            return 0;
        }

        std::vector<vpImagePoint> corners = polygons[0];

        // -------------------------------
        // 6. Convert to normalized coords
        // -------------------------------
        std::vector<vpFeaturePoint> features;

        for (size_t i = 0; i < corners.size(); i++) {

            double u = corners[i].get_u();
            double v = corners[i].get_v();

            // Normalize using camera intrinsics
            double x = (u - cam.get_u0()) / cam.get_px();
            double y = (v - cam.get_v0()) / cam.get_py();

            vpFeaturePoint s;
            s.buildFrom(x, y, 1);

            features.push_back(s);

            std::cout << "Corner " << i
                << " | Pixel: (" << u << ", " << v << ")"
                << " | Normalized: (" << x << ", " << y << ")"
                << std::endl;
        }

        std::cout << "\nDetection + normalization SUCCESS " << std::endl;
    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
    }
    catch (...) {
        std::cout << "Unknown error occurred!" << std::endl;
    }

    return 0;
}

