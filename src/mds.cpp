#include "mds.h"


Eigen::MatrixXd compute_gramm_matrix(const MatrixXd& D) {
	// return the gramm matrix associated to D here
	// complete here
	int n = D.rows();
	MatrixXd G;

	return G;
}



Eigen::MatrixXd compute_new_embeding(const MatrixXd& G, int d = 2) {
	//compute Z using G's eigenvectors
    int n = G.rows();
	MatrixXd Z(n,d);

	return Z;
  }
