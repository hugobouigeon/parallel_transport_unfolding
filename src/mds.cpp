#include "mds.h"


Eigen::MatrixXd compute_gramm_matrix(MatrixXd& D) {
	// return the gramm matrix associated to D here
	int n = D.rows();
	MatrixXd G;
	MatrixXd I = MatrixXd::Identity(n, n);
	MatrixXd Ones = MatrixXd::Ones(n, n);
	
	G = -0.5 * (I - Ones / n) * D * (I - Ones / n);
	return G;
}



Eigen::MatrixXd compute_new_embedding(MatrixXd& G, int d) {
	//compute Z using G's eigenvectors
    int n = G.rows();

	MatrixXd Z(n, d);

	SelfAdjointEigenSolver<MatrixXd> es(G);
	MatrixXd eigenvectors = es.eigenvectors().real();
	MatrixXd eigenvalues = es.eigenvalues().real();

	for (int i = 0; i < d; i++) {
		Z.col(i) = std::sqrt(eigenvalues(n-1-i)) * eigenvectors.col(n-1-i).transpose();
	}

	return Z;
  }
