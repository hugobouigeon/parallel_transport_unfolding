#include "prll_transport.h"


// typedef std::pair<double, std::vector<int > > path;
typedef std::pair<double, int > path; // contains path length and vertex

std::vector<Eigen::MatrixXd > Tangent_spaces;

// TODO might want to split this function
void dijkstra(const MatrixXd &V, const Eigen::MatrixXi &I, Eigen::MatrixXd &Dist, int src, int k) {
	int n = V.rows();

	// contains current shortest path length and predecessor in spanning tree
	path pred[n];
	double distances_to_src[n];
	for (int i = 0; i < n; i++) {
		pred[i] = std::make_pair(-1, -1);
	}
	pred[src] = std::make_pair(0, src);
	distances_to_src[src] = 0;

	// contains path length and end vertex, use preds array for predecessor
	std::priority_queue<path, std::vector<path >, std::greater<path > > pq; // TODO check that it works lol
	for (int i = 0; i < k; i++) {
		int v = I(src, i);
		double dist_to_vertex = (V.row(src) - V.row(v)).norm();
		pred[v] = std::make_pair(dist_to_vertex, src);
		pq.push(std::make_pair(dist_to_vertex, v));
	}

	while(!pq.empty()) {
		path p = pq.top();
		pq.pop();
		int v = p.second;
		if (distances_to_src[v] >= 0) continue; // vertex already processed
		distances_to_src[v] = p.first;

		// adding new outgoing edges to priority queue
		for (int i = 0; i < k; i++) {
			int new_v = I(v, i);
			if (distances_to_src[new_v] < 0) {
				double path_length = p.first + (V.row(v) - V.row(new_v)).norm();
				if (pred[new_v].first < 0 || pred[new_v].first > path_length) {
					pred[new_v] = std::make_pair(path_length, v);
					pq.push(std::make_pair(path_length, new_v));
				}
			}
		}
	}

	// TODO compute actual geodesic distances with unfolding
	for (int i = 0; i < n; i++) {
		MatrixXd geodesic_path;
	}
}

Eigen::MatrixXd compute_Rij(const MatrixXd& Ti, const Eigen::MatrixXd& Tj) {
	// compute Rij using SVD
	// cf. eq (3) p.6 of PTU paper
	JacobiSVD<MatrixXd> svd(Tj.transpose() * Ti, Eigen::ComputeFullU | Eigen::ComputeFullV);

	MatrixXd U = svd.matrixU();
	MatrixXd V = svd.matrixV();

	return U * V.transpose();
}

//TODO dico de Rij?

/**
 * Computes the distance matrix D between points of the point cloud
 * Uses the Dijktra algorithm to find a path and then unfolding to estimate the geodesic
*/
Eigen::MatrixXd compute_distance_matrix(const MatrixXd &V, const int k, const int d){
	int n = V.rows();
	Tangent_spaces.resize(n);
	MatrixXd Dist(n, n);

	Eigen::MatrixXi I;
	k_nearest_neighbour(V,I,k);

	for (int i = 0; i < n; i++) {
		Tangent_spaces[i] = compute_tangent_space(V, I, k, d, i);
	}
	for (int i = 0; i<n; i++) {
		dijkstra(V, I, Dist, i, k);
	}
	// Post-processing to correct potential asymmetries
	Dist = (Dist + Dist.transpose()) / 2;
	return Dist;
  }
