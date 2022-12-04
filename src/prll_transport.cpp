#include "prll_transport.h"
#include <map>

typedef std::pair<double, int > path; // contains path length and vertex

std::vector<Eigen::MatrixXd > Tangent_spaces;
std::map<std::pair<int, int>, Eigen::MatrixXd > Rij_values;

std::pair<std::vector<int >, std::vector<int > > dijkstra(const MatrixXd &V, MatrixXi &I, int src, int k) {
	int n = V.rows();

	// contains current shortest path length and predecessor in spanning tree
	std::vector<int > predecessor(n);
	std::vector<double > current_distance(n);
	std::vector<double > distances_to_src(n);
	for (int i = 0; i < n; i++) {
		predecessor[i] = -1;
		current_distance[i] = -1;
		distances_to_src[i] = -1;
	}
	predecessor[src] = src;
	current_distance[src] = 0;
	distances_to_src[src] = 0;

	// contains path length and end vertex, use preds array for predecessor
	std::priority_queue<path, std::vector<path >, std::greater<path > > pq; // TODO check that it works lol
	std::vector<int > order; // order in which to process vertices for unfolding
	for (int i = 0; i < k; i++) {
		int v = I(src, i);
		if (v == src) continue;
		double dist_to_vertex = (V.row(src) - V.row(v)).norm();
		predecessor[v] = src;
		current_distance[v] = dist_to_vertex;
		pq.push(std::make_pair(dist_to_vertex, v));
	}

	while(!pq.empty()) {
		path p = pq.top();
		pq.pop();
		int v = p.second;
		if (distances_to_src[v] >= 0) continue; // vertex already processed
		distances_to_src[v] = p.first;
		order.push_back(v);

		// adding new outgoing edges to priority queue
		for (int i = 0; i < k; i++) {
			int new_v = I(v, i);
			if (v == new_v) continue;
			if (distances_to_src[new_v] < 0) {
				double path_length = p.first + (V.row(v) - V.row(new_v)).norm();
				if (current_distance[new_v] < 0 || current_distance[new_v] > path_length) {
					predecessor[new_v] = v;
					current_distance[new_v] = path_length;
					pq.push(std::make_pair(path_length, new_v));
				}
			}
		}
	}
	return std::make_pair(order, predecessor); // order could be recomputed but it would be costly
}

void compute_unfolding(const MatrixXd &V, std::pair<std::vector<int >, std::vector<int > > &geo_path, MatrixXd &Dist, int src, int d) {
	int n = V.rows();
	Dist(src, src) = 0;
	MatrixXd projected_points = MatrixXd::Zero(n, d);
	std::vector<int > order = geo_path.first;
	std::vector<int > predecessors = geo_path.second;
	// TODO length rescaling ?
	for (int i = 0; i < n-1; i++) {
		int v = order[i];
		int pred = predecessors[v];
		VectorXd ei = Tangent_spaces[pred].transpose() * (V.row(v) - V.row(pred)).transpose();
		while (pred != src) {
			int next_pred = predecessors[pred];
			ei = get_Rij(next_pred, pred) * ei;
			pred = next_pred;
		}
		projected_points.row(v) = projected_points.row(predecessors[v]) + ei.transpose();
		double path_len = projected_points.row(v).norm();
		Dist(src, v) = path_len * path_len;
	}
}

Eigen::MatrixXd get_Rij(int i, int j) {
	MatrixXd Rij;
	if (Rij_values.find(std::make_pair(i, j)) == Rij_values.end()) {
		Rij = compute_Rij(Tangent_spaces[i], Tangent_spaces[j]);
		Rij_values[std::make_pair(i, j)] = Rij;
		Rij_values[std::make_pair(j, i)] = Rij.transpose();
	} else {
		Rij = Rij_values.find(std::make_pair(i, j))->second;
	}
	return Rij;
}

Eigen::MatrixXd compute_Rij(MatrixXd& Ti, Eigen::MatrixXd& Tj) {
	// compute Rij using SVD
	// cf. eq (3) p.6 of PTU paper
	JacobiSVD<MatrixXd> svd(Tj.transpose() * Ti, Eigen::ComputeFullU | Eigen::ComputeFullV);

	MatrixXd U = svd.matrixU();
	MatrixXd V = svd.matrixV();
	return U * V.transpose();
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
	for (int i = 0; i < n; i++) {
		Tangent_spaces[i] = compute_tangent_space(V, I, k, d, i);
	}
	std::cout << "Computing the distance matrix:" << std::endl;
	for (int i = 0; i<n; i++) {
		int barWidth = 70;
		float progress = static_cast<float>(i)/static_cast<float>(V.rows());
		std::cout << "[";
		int pos = barWidth * progress;
		for (int i = 0; i < barWidth; ++i) {
			if (i < pos) std::cout << "=";
			else if (i == pos) std::cout << ">";
			else std::cout << " ";
		}
		std::cout << "] " << int(progress * 100.0) << " %\r";
		std::cout.flush();

		auto geo_path = dijkstra(V, I, i, k);
		compute_unfolding(V, geo_path, Dist, i, d);
	}
	std::cout.flush();
	std::cout << "Computing the distance matrix: Done                                           " << std::endl;
	// Post-processing to correct potential asymmetries
	Dist = (Dist + Dist.transpose()) / 2;
	return Dist;
  }
