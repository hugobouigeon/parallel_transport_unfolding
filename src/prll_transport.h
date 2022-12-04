#include <Eigen/Core>
using namespace Eigen;
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <igl/octree.h>
#include <igl/knn.h>
#include "pca.h"

Eigen::MatrixXd compute_Rij(MatrixXd& Ti, Eigen::MatrixXd& Tj);
Eigen::MatrixXd compute_distance_matrix(const MatrixXd& V, int k, int d);