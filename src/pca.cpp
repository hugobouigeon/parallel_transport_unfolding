#include "pca.h"


void k_nearest_neighbour(const MatrixXd& V, Eigen::MatrixXi& I, int k) {
	// return the k nearest neighbour index
	int n = V.rows();
	std::vector<std::vector<int > > O_PI;
	Eigen::MatrixXi O_CH;
	Eigen::MatrixXd O_CN;
	Eigen::VectorXd O_W;
	igl::octree(V, O_PI, O_CH, O_CN, O_W);
	Eigen::VectorXd A;
	{
		igl::knn(V, k, O_PI, O_CH, O_CN, O_W, I);
	}
}

// void compute_normals(const MatrixXd &V,const Eigen::MatrixXi &I, int k, MatrixXd &normals){
//     // compute the normals using PCA
// 	MatrixXd out(V.rows(), 3);

//     int n = V.rows();
	
// 	for (int i = 0; i < n; i++) {

// 		MatrixXd C(3, 3);
// 		C(0, 0) = 0;	C(0, 1) = 0;	C(0, 2) = 0;
// 		C(1, 0) = 0;	C(1, 1) = 0;	C(1, 2) = 0;
// 		C(2, 0) = 0;	C(2, 1) = 0;	C(2, 2) = 0;

// 		Vector3d P = V.row(i);
// 		for (int j = 0; j < k; j++) {
// 			Vector3d pj = V.row(I(i,j));
// 			if (pj != P) {
// 				C += (pj - P) * (pj - P).transpose();
// 			}
// 		}
// 		SelfAdjointEigenSolver<Matrix3d> es(C);
// 		MatrixXcd eigenvectors = es.eigenvectors();

// 		out(i, 0) = eigenvectors(0, 0).real()/5;
// 		out(i, 1) = eigenvectors(1, 0).real()/5;
// 		out(i, 2) = eigenvectors(2, 0).real()/5;
// 	}
// 	normals = out;
// }

Eigen::MatrixXd compute_tangent_space(const MatrixXd &V, Eigen::MatrixXi &I, int k, int d, int i) {
	// computes an orthonormal basis for the local tangent space defined by the k closest neighbors of each point
	int D = V.cols(); // initial number of dimensions
	MatrixXd N = MatrixXd::Zero(D, k);
	bool seen_self = false;
	for (int j = 0; j < k; j++) {
		if (I(i, j) == i) {
			seen_self = true;
		}
		else {
			int neighbor_idx = seen_self ? j-1 : j;
			N.col(neighbor_idx) = (V.row(I(i,j)) - V.row(i));
		}
	}
	JacobiSVD<MatrixXd> es(N, ComputeThinU | ComputeThinV);
	// MatrixXd U = svd.matrixU();
	MatrixXd eigenvectors = es.matrixV().real();

	// SelfAdjointEigenSolver<MatrixXd> es(N);
	// MatrixXd eigenvectors = es.eigenvectors().real();
	MatrixXd Ti = MatrixXd::Zero(D, d);
	for (int j = 0; j < d; j++) {
		Ti.col(j) = eigenvectors.row(j);
	}
	return Ti;
}