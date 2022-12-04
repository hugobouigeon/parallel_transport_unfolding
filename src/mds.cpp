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
	MatrixXd Lambda = MatrixXd::Zero(n,n);

	// float lambda = 1;

	JacobiSVD<MatrixXd> es(G, ComputeThinU | ComputeThinV);

	MatrixXd eigenvectors = es.matrixV().real();
	MatrixXd eigenvalues = es.singularValues().real();

	for (int i = 0; i < d; i++) {
		Q.col(i) = std::sqrt(eigenvalues(i)) * eigenvectors.row(i).transpose();
		Lambda(i,i) = std::sqrt(eigenvalues(i));
	}

	Z = Q;

	return Z;
  }
