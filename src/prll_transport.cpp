#include "prll_transport.h"


Eigen::MatrixXd compute_Rij(const MatrixXd& Ti, Eigen::MatrixXd& Tj) {
	// compute Rij using SVD
	// cf. eq (3) p.6 of PTU paper
	JacobiSVD<MatrixXd> svd(Tj.transpose() * Ti, Eigen::ComputeFullU | Eigen::ComputeFullV);

	MatrixXd U = svd.matrixU();
	MatrixXd V = svd.matrixV();

	return U * V.transpose();
}

//TODO dico de Rij?

Eigen::MatrixXd compute_D(const MatrixXd &V1,const Eigen::MatrixXi &I, MatrixXd &normals){
	// compute D the distance matrix of the pointcloud
	int n = V1.rows();
	MatrixXd D(V1.rows(), 3);
	
	return D;
  }
