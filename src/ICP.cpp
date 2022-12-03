#include "ICP.h"

void nearest_neighbour(const MatrixXd &V1, const MatrixXd &V2, MatrixXd &nn_V2){
  // return the nearest neighbour to V1 in V2 as nn_V2
  // Complete here
	int n = V1.rows();
	int m = V2.rows();
	float dist = 0;
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < m; j++) {
			if ((nn_V2.row(i) - V1.row(i)).norm() > (V2.row(j) - V1.row(i)).norm()) {
				nn_V2.row(i) = V2.row(j);
			}
		}
		dist += (nn_V2.row(i) - V1.row(i)).norm();
	}
	std::cout << dist << std::endl;
}


void transform(MatrixXd &V1,const MatrixXd &V2){
  //align V1 to V2 when V1 and V2 points are in correspondance
  // complete here
	int n = V1.rows();

	MatrixXd v1mean = V1.colwise().mean();
	MatrixXd v2mean = V2.colwise().mean();
	MatrixXd t = v2mean - v1mean;

	MatrixXd C(3, 3);
	C(0, 0) = 0;	C(0, 1) = 0;	C(0, 2) = 0;
	C(1, 0) = 0;	C(1, 1) = 0;	C(1, 2) = 0;
	C(2, 0) = 0;	C(2, 1) = 0;	C(2, 2) = 0;

	for (int i = 0; i < n; i++) {

		C += (V1.row(i) - v1mean).transpose() * (V2.row(i) - v2mean);
	}

	JacobiSVD<MatrixXd> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);

	MatrixXd U = svd.matrixU();
	MatrixXd V = svd.matrixV();

	MatrixXd Ropt;


	if ((V*U.transpose()).determinant() > 0) {
		Ropt = V * U.transpose();
	}
	else {
		MatrixXd diag(3, 3);
		diag(0, 0) = 1; diag(1, 1) = 1; diag(2, 2) = -1;
		Ropt = V * diag * U.transpose();
	}


	V1 = V1 * Ropt.transpose();
	for (int i = 0; i < n; i++) {
		V1.row(i) += t;
	}

}
