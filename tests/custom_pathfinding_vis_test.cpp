#include <Eigen/Dense>
#include <vector>
#include "test_helpers.hpp"

extern "C" {

// Function to generate a random valid Cartesian position (placeholder implementation)
void random_valid_cart_pos(double* pos) {
auto vec = random_valid_cart_pos();
    pos[0] = vec.x();
    pos[1] = vec.y();
    pos[2] = vec.z();
}

// Function to find a path from start_pos to goal_pos
double* find_path(double* start_pos, double* goal_pos, int* path_size) {
    Eigen::Vector3d start(start_pos[0], start_pos[1], start_pos[2]);
    Eigen::Vector3d goal(goal_pos[0], goal_pos[1], goal_pos[2]);

    // Example path (placeholder implementation)
    std::vector<Eigen::Vector3d> path;
    path.push_back(start);
    path.push_back((start + goal) / 2); // Midpoint
    path.push_back(goal);

    *path_size = path.size() * 3; // Total number of doubles

    double* result = new double[*path_size];
    for (int i = 0; i < path.size(); ++i) {
        result[i * 3 + 0] = path[i].x();
        result[i * 3 + 1] = path[i].y();
        result[i * 3 + 2] = path[i].z();
    }

    return result;
}

// Function to free allocated memory
void free_memory(double* array) {
    delete[] array;
}

}
