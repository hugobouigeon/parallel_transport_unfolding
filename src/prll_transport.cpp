#include "prll_transport.h"


Eigen::MatrixXd compute_Rij(const MatrixXd& T1, Eigen::MatrixXi& T2) {
	// compute Rij using SVD
	// complete here
	//MatrixXi out(V1.rows(), k);
	MatrixXd R;
	return R;
}

//TODO dico de Rij?

Eigen::MatrixXd compute_D(const MatrixXd &V1,const Eigen::MatrixXi &I, MatrixXd &normals){
	// compute D the distance matrix of the pointcloud
	int n = V1.rows();
	MatrixXd D(V1.rows(), 3);
	
	return D;
  }
