#include <Eigen/Core>
using namespace Eigen;
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <igl/octree.h>
#include <igl/knn.h>

void k_nearest_neighbour(const MatrixXd &V, Eigen::MatrixXi &I, int k);
Eigen::MatrixXd compute_tangent_space(const MatrixXd &V, Eigen::MatrixXi &I, int k, int d, int i);