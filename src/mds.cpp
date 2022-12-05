#include "mds.h"


Eigen::MatrixXd compute_gramm_matrix(MatrixXd& D) {
	// return the gramm matrix associated to D here
	// complete here
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
	MatrixXd Q(n, d);

	// float lambda = 1;

	// JacobiSVD<MatrixXd> es(G, ComputeThinU | ComputeThinV);
	SelfAdjointEigenSolver<MatrixXd> es(G);
	MatrixXd eigenvectors = es.eigenvectors().real();

	// MatrixXd eigenvectors = es.matrixV().real();
	MatrixXcd eigenvalues = es.eigenvalues().real();
	// for (int i = 0; i < n; i++) {
	// 	std::cout << ev(i) << std::endl;
	// }
	// MatrixXd eigenvalues = ev.real();

	for (int i = 0; i < d; i++) {
		std::cout << eigenvalues(d-1-i) << std::endl;
		Q.col(i) = std::sqrt(abs(eigenvalues(d-1-i))) * eigenvectors.row(d-1-i).transpose();
	}

	Z = Q;

	return Z;
  }
