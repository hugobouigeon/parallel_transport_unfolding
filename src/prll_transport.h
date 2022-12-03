#include <Eigen/Core>
using namespace Eigen;
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <igl/octree.h>
#include <igl/knn.h>

Eigen::MatrixXd compute_Rij(const MatrixXd& T1, Eigen::MatrixXi& T2);
Eigen::MatrixXd compute_D(const MatrixXd& V1, const Eigen::MatrixXi& I, MatrixXd& normals);
