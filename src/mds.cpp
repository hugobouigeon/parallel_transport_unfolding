#include "mds.h"


Eigen::MatrixXd compute_gramm_matrix(MatrixXd& D) {
	// return the gramm matrix associated to D here
	// complete here
	int n = D.rows();
	MatrixXd G;
	MatrixXd I = MatrixXd::Identity(n, n);
	MatrixXd Ones = MatrixXd::Ones(n, n);
	
	G = -0.5 * (I - Ones) * D * (I - Ones);
	return G;
}



Eigen::MatrixXd compute_new_embedding(MatrixXd& G, int d) {
	//compute Z using G's eigenvectors
    int n = G.rows();

	MatrixXd Z(n, d);
	MatrixXd Q(n, d);

	float lambda = 1;

	JacobiSVD<MatrixXd> es(G, ComputeThinU | ComputeThinV);

	MatrixXd eigenvectors = es.matrixV().real();
	MatrixXd eigenvalues = es.singularValues().real();

	for (int i = 0; i < d; i++) {
		Q.col(i) = eigenvectors.row(i).transpose();
		lambda *= eigenvalues(i);
	}

	Z = std::sqrt(lambda) * Q;

	return Z;
  }
