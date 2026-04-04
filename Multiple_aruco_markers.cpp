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
        vpCameraParameters cam(800, 800, 320, 240);

        vpDetectorAprilTag detector(vpDetectorAprilTag::TAG_36h11);
        detector.setAprilTagQuadDecimate(1.0);

        std::vector<std::string> image_list = {
            "C:\\Users\\Hp\\source\\repos\\Visptest\\x64\\Release\\marker.jpg",
            "C:\\Users\\Hp\\source\\repos\\Visptest\\x64\\Release\\marker_zoomedin70.jpg",
            "C:\\Users\\Hp\\source\\repos\\Visptest\\x64\\Release\\marker_zoomedin.jpg",
            "C:\\Users\\Hp\\source\\repos\\Visptest\\x64\\Release\\marker_30deg.jpg"
        };

        for (size_t img_idx = 0; img_idx < image_list.size(); img_idx++) {

            vpImage<unsigned char> I;
            vpImageIo::read(I, image_list[img_idx]);

            bool detected = detector.detect(I);

            if (!detected) {
                std::cout << "Marker not detected in " << image_list[img_idx] << std::endl;
                continue;
            }

            std::vector<std::vector<vpImagePoint>> polygons = detector.getPolygon();
            std::vector<vpImagePoint> corners = polygons[0];

            std::cout << "\nImage: " << image_list[img_idx] << std::endl;

            for (size_t i = 0; i < corners.size(); i++) {

                double u = corners[i].get_u();
                double v = corners[i].get_v();

                std::cout << "Corner " << i
                    << " | Pixel: (" << u << ", " << v << ")"
                    << std::endl;
            }
        }
    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
    }

    return 0;
}
