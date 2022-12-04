#include "prll_transport.h"
#include <map>


// typedef std::pair<double, std::vector<int > > path;
typedef std::pair<double, int > path; // contains path length and vertex

std::vector<Eigen::MatrixXd > Tangent_spaces;
std::map<std::pair<int, int>, Eigen::MatrixXd > Rij_values;

// TODO might want to split this function
std::pair<std::vector<int >, std::vector<int > > dijkstra(const MatrixXd &V, MatrixXi &I, int src, int k) {
	int n = V.rows();

	// contains current shortest path length and predecessor in spanning tree
	// path pred[n];
	std::vector<int > predecessor(n);
	std::vector<double > current_distance(n);
	std::vector<double > distances_to_src(n);
	// double distances_to_src[n];
	for (int i = 0; i < n; i++) {
		// pred[i] = std::make_pair(-1, -1);
		predecessor[i] = -1;
		current_distance[i] = -1;
		distances_to_src[i] = -1;
	}
	predecessor[src] = src;
	current_distance[src] = 0;
	// pred[src] = std::make_pair(0, src);
	distances_to_src[src] = 0;

	// std::cout << "Test 5" << std::endl;

	// contains path length and end vertex, use preds array for predecessor
	std::priority_queue<path, std::vector<path >, std::greater<path > > pq; // TODO check that it works lol
	std::vector<int > order; // order in which to process vertices for unfolding
	for (int i = 0; i < k; i++) {
		int v = I(src, i);
		double dist_to_vertex = (V.row(src) - V.row(v)).norm();
		// pred[v] = std::make_pair(dist_to_vertex, src);
		predecessor[v] = src;
		current_distance[v] = dist_to_vertex;
		pq.push(std::make_pair(dist_to_vertex, v));
	}

	// std::cout << "Test 4" << std::endl;

	while(!pq.empty()) {
		path p = pq.top();
		pq.pop();
		int v = p.second;
		if (distances_to_src[v] >= 0) continue; // vertex already processed
		distances_to_src[v] = p.first;
		order.push_back(v);
		// std::cout << v << std::endl;

		// adding new outgoing edges to priority queue
		for (int i = 0; i < k; i++) {
			int new_v = I(v, i);
			if (distances_to_src[new_v] < 0) {
				double path_length = p.first + (V.row(v) - V.row(new_v)).norm();
				// if (pred[new_v].first < 0 || pred[new_v].first > path_length) {
				if (current_distance[new_v] < 0 || current_distance[new_v] > path_length) {
					// pred[new_v] = std::make_pair(path_length, v);
					// std::cout << "yo" << std::endl;
					predecessor[new_v] = v;
					current_distance[new_v] = path_length;
					pq.push(std::make_pair(path_length, new_v));
				}
			}
		}
	}
	std::cout << "HEYYYYYYYYYYYYYYYYYY" << std::endl;
	// for (int i = 0; i < n; i++){
	// 	std::cout << predecessor[i] << std::endl;
	// }
	return std::make_pair(order, predecessor); // order could be recomputed but it would be costly
}

void compute_unfolding(const MatrixXd &V, std::pair<std::vector<int >, std::vector<int > > &geo_path, MatrixXd &Dist, int src, int d) {
	int n = V.rows();
	Dist(src, src) = 0;
	MatrixXd projected_points = MatrixXd::Zero(n, d);
	std::vector<int > order = geo_path.first;
	std::vector<int > predecessors = geo_path.second;
	// TODO length rescaling ?
	// std::cout << "Test 5" << std::endl;
	for (int i = 0; i < n-1; i++) {
		// std::cout << "i = " << i << std::endl;
		int v = order[i];
		int pred = predecessors[i];
		// std::cout << "Test 12" << std::endl;
		// std::cout << "v = " << v << ", pred = " << pred << std::endl;
		// std::cout << V.row(v) - V.row(pred) << std::endl;
		// std::cout << std::endl;
		// std::cout << Tangent_spaces[pred].transpose() << std::endl;
		VectorXd ei = Tangent_spaces[pred].transpose() * (V.row(v) - V.row(pred)).transpose();
		// std::cout << ei.transpose() << std::endl;
		// std::cout << "Test 6" << std::endl;
		while (pred != src) {
			int next_pred = predecessors[pred];
			ei = compute_Rij(Tangent_spaces[next_pred], Tangent_spaces[pred]) * ei;
			// std::cout << "Test 7" << std::endl;
			pred = next_pred;
		}
		projected_points.row(i) = projected_points.row(predecessors[i]) + ei.transpose();
		// std::cout << "Test 11" << std::endl;
		Dist(src, i) = projected_points.row(i).norm();
	}
}

Eigen::MatrixXd get_Rij(int i, int j) {
	return MatrixXd::Zero(10,10);
}

Eigen::MatrixXd compute_Rij(MatrixXd& Ti, Eigen::MatrixXd& Tj) {
	// compute Rij using SVD
	// cf. eq (3) p.6 of PTU paper
	// std::cout << "Test 10" << std::endl;
	JacobiSVD<MatrixXd> svd(Tj.transpose() * Ti, Eigen::ComputeFullU | Eigen::ComputeFullV);

	MatrixXd U = svd.matrixU();
	MatrixXd V = svd.matrixV();
	// std::cout << "Test 8" << std::endl;
	return U * V.transpose();
	// std::cout << "Test 9" << std::endl;
}

/**
 * Computes the distance matrix D between points of the point cloud
 * Uses the Dijktra algorithm to find a path and then unfolding to estimate the geodesic
*/
Eigen::MatrixXd compute_distance_matrix(const MatrixXd &V, int k, int d){
	int n = V.rows();
	Tangent_spaces.resize(n);
	MatrixXd Dist(n, n);

	Eigen::MatrixXi I;
	k_nearest_neighbour(V,I,k);
	std::cout << "Test 1" << std::endl;
	for (int i = 0; i < n; i++) {
		Tangent_spaces[i] = compute_tangent_space(V, I, k, d, i);
	}
	std::cout << "Test 2" << std::endl;
	for (int i = 0; i<n; i++) {
		std::cout << "i = " << i << "/ " << V.rows() << std::endl;
		auto geo_path = dijkstra(V, I, i, k);
		std::cout << "Test 3" << std::endl;
		compute_unfolding(V, geo_path, Dist, i, d);
		std::cout << "Test 4" << std::endl;
	}
	// Post-processing to correct potential asymmetries
	Dist = (Dist + Dist.transpose()) / 2;
	return Dist;
  }
