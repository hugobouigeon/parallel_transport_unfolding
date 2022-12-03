#include "pca.h"


void k_nearest_neighbour(const MatrixXd& V1, Eigen::MatrixXi& I, int k) {
	// return the k nearest neighbour index
	// complete here
	//MatrixXi out(V1.rows(), k);
	std::cout << "debug1" << std::endl;
	int n = V1.rows();
	std::vector<std::vector<int > > O_PI;
	Eigen::MatrixXi O_CH;
	Eigen::MatrixXd O_CN;
	Eigen::VectorXd O_W;
	igl::octree(V1, O_PI, O_CH, O_CN, O_W);
	Eigen::VectorXd A;
	{
		igl::knn(V1, k, O_PI, O_CH, O_CN, O_W, I);
		// CGAL is only used to help get point areas
		//igl::copyleft::cgal::point_areas(V1, I, N, A);
	}


	

}



void compute_normals(const MatrixXd &V1,const Eigen::MatrixXi &I, int k, MatrixXd &normals){
  // compute the normals using PCA
	MatrixXd out(V1.rows(), 3);

    int n = V1.rows();
	
	for (int i = 0; i < n; i++) {

		MatrixXd C(3, 3);
		C(0, 0) = 0;	C(0, 1) = 0;	C(0, 2) = 0;
		C(1, 0) = 0;	C(1, 1) = 0;	C(1, 2) = 0;
		C(2, 0) = 0;	C(2, 1) = 0;	C(2, 2) = 0;

		Vector3d P = V1.row(i);
		for (int j = 0; j < k; j++) {
			Vector3d pj = V1.row(I(i,j));
			if (pj != P) {
				C += (pj - P) * (pj - P).transpose();
			}
		}
		SelfAdjointEigenSolver<Matrix3d> es(C);
		MatrixXcd eigenvectors = es.eigenvectors();

		out(i, 0) = eigenvectors(0, 0).real()/5;
		out(i, 1) = eigenvectors(1, 0).real()/5;
		out(i, 2) = eigenvectors(2, 0).real()/5;
		
		
	}
	normals = out;
	
  }
