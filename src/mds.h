#include <Eigen/Core>
using namespace Eigen;
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <igl/octree.h>
#include <igl/knn.h>

Eigen::MatrixXd compute_gramm_matrix(Eigen::MatrixXd& D);
Eigen::MatrixXd compute_new_embedding(MatrixXd& G, int d);
