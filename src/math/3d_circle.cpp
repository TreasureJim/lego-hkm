#include <Eigen/src/Core/Matrix.h>


Eigen::Vector3d find_plane_normal(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) {
	boost::container::vec
	
	Eigen::Vector3d v1 = p2 - p1;
	Eigen::Vector3d v2 = p3 - p1;
	Eigen::Vector3d normal = v1.cross(v2);

}

void sadjfks(Eigen::Vector3d p1, Eigen::Vector3d p2, Eigen::Vector3d p3) {
	// Eigen::Vector3d normal = 
}
