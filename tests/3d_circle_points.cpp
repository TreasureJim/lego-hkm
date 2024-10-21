#include "lego_model.hpp"
#include "math/3d_circle.hpp"
#include <Eigen/Core>
#include <iostream>
#include <string>

void print_vec(Eigen::Vector3d p) {
    std::cout << "Point: " << p.x() << ", "  << p.y() << ", " << p.z() << std::endl;
}

int main (int argc, char *argv[]) {
    
    Eigen::Vector3d p1;
		p1[0] = std::stod(argv[1]);
		p1[1] = std::stod(argv[2]);
		p1[2] = std::stod(argv[3]);

    Eigen::Vector3d p2;
		p2[0] = std::stod(argv[1]);
		p2[1] = std::stod(argv[2]);
		p2[2] = std::stod(argv[3]);

    Eigen::Vector3d p3;
		p3[0] = std::stod(argv[1]);
		p3[1] = std::stod(argv[2]);
		p3[2] = std::stod(argv[3]);

    std::cout << "Start" << std::endl;
    print_vec(p1);
    std::cout << "aPos" << std::endl;
    print_vec(p2);
    std::cout << "End" << std::endl;
    print_vec(p3);
    std::cout << "\n\n\n" << std::endl;

    auto circle = Circle_3D(&lego_model, p1, p2, p3);

    for (double p = 0.0; p <= 2 * M_PI; p += 0.1) {
        print_vec(circle.get_circle_coord(p));
    }


    return 0;
}
