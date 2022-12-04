#include <Eigen/Core>
using namespace Eigen;
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <igl/octree.h>
#include <igl/knn.h>
#include "pca.h"

Eigen::MatrixXd compute_Rij(const MatrixXd& Ti, const Eigen::MatrixXd& Tj);
Eigen::MatrixXd compute_D(const MatrixXd& V, const int k);
