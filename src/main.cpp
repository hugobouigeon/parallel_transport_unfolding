#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOFF.h>
#include <igl/readPLY.h>
#include <igl/octree.h>
#include <igl/knn.h>
#include <igl/writeOBJ.h>
#include <iostream>
#include <ostream>
#include <igl/jet.h>
#include <fstream>

#include "mds.h"
#include "prll_transport.h"
// #include "ICP.h"

//using namespace Eigen; // to use the classes provided by Eigen library
MatrixXd V; // matrix storing vertex coordinates of the input curve
MatrixXi F1;
// MatrixXd V2; // matrix storing vertex coordinates of the input curve
// MatrixXi F2;


bool key_down(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifier) {
  std::cout << "pressed Key: " << key << " " << (unsigned int)key << std::endl;

  // if (key == '1')
  // {
  //   transform(V,V2);
  //   viewer.data(0).clear(); // Clear should be called before drawing the mesh
  //   viewer.data(0).set_mesh(V, F1);
  //   viewer.data(0).set_colors(Eigen::RowVector3d(0.3, 0.8, 0.3));// update the mesh (both coordinates and faces)
  // }

  return false;
}

void draw_normals(igl::opengl::glfw::Viewer &viewer, const MatrixXd &V, const MatrixXd &n){
  MatrixXd current_edge(V.rows(), 3);
  for (unsigned i = 1; i < V.rows()-1; ++i){
    current_edge(i,0) =  V(i, 0) + n(i, 0);
    current_edge(i,1) =  V(i, 1)+ n(i, 1);
    current_edge(i,2) =  V(i, 2)+ n(i, 2);
    viewer.data().add_edges(
        V.row(i),
        current_edge.row(i),
        Eigen::RowVector3d(1, 1, 1));
    }
}

void set_meshes(igl::opengl::glfw::Viewer &viewer) {
  viewer.callback_key_down = &key_down; // for dealing with keyboard events
  viewer.data().set_mesh(V, F1);
  // viewer.append_mesh();
  // viewer.data().set_mesh(V2, F2);
  // viewer.data(0).set_colors(Eigen::RowVector3d(0.3, 0.8, 0.3));
  // viewer.data(1).set_colors(Eigen::RowVector3d(0.8, 0.3, 0.3));
}

void set_pc(igl::opengl::glfw::Viewer &viewer) {
  viewer.callback_key_down = &key_down; // for dealing with keyboard events
  viewer.data().add_points(V,Eigen::RowVector3d(0.3, 0.8, 0.3));
}

// void ex1() {
//   igl::readOFF("../data/star.off", V, F1);
//   igl::readOFF("../data/star_rotated.off", V2, F2);
//   igl::opengl::glfw::Viewer viewer;
//   set_meshes(viewer);
//   viewer.launch();
// }

// void ex2() {
//   igl::readOFF("../data/bunny.off", V, F1);
//   igl::readOFF("../data/bunny_rotated.off", V2, F2);
//   igl::opengl::glfw::Viewer viewer;
//   set_meshes(viewer);
//     viewer.core().is_animating = true;
//     viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer & )->bool // run animation
//     {
//       Eigen::MatrixXd nn_V2(V.rows(), V.cols());
//       nearest_neighbour(V, V2, nn_V2);
//       //  complete here by displaying the pair wise distances
//       transform(V,nn_V2);
//       viewer.data(0).clear(); // Clear should be called before drawing the mesh
//       viewer.data(0).set_mesh(V, F1);
//       viewer.data(0).set_colors(Eigen::RowVector3d(0.3, 0.8, 0.3));// update the mesh (both coordinates and faces)}
//       return false; };
//       viewer.launch(); // run the editor
// }

// void ex3(){
//   igl::readOFF("../data/swiss_roll_noise_0.2.off", V, F1);
//   igl::opengl::glfw::Viewer viewer;
//   set_pc(viewer);
//   Eigen::MatrixXi I;
//   k_nearest_neighbour(V,I,12);
//   Eigen::MatrixXd normals(V.rows(), 3);
//   Eigen::MatrixXd A(V.rows(), 3);
//   Eigen::MatrixXd B(V.rows(), 3);
//   compute_normals(V,I, 12, normals);
//   draw_normals(viewer, V, normals);
//   viewer.launch();
// }



int main(int argc, char *argv[])
{
    /*
    Eigen::MatrixXd D(4, 4);
    D(0, 0) = 0; D(0, 1) = 1; D(0, 2) = 3; D(0, 3) = 4;
    D(1, 0) = 1; D(1, 1) = 0; D(1, 2) = 2; D(1, 3) = 3;
    D(2, 0) = 3; D(2, 1) = 2; D(2, 2) = 0; D(2, 3) = 1;
    D(3, 0) = 4; D(3, 1) = 3; D(3, 2) = 1; D(3, 3) = 0;
    MatrixXd G = compute_gramm_matrix(D);
    MatrixXd Z = compute_new_embedding(G,2);
    */
  std::string input = argc < 2 ? "../data/petit_swiss_roll.off" : argv[1];
  bool usePTU = argc < 3;
  igl::readOFF(input, V, F1);
  igl::opengl::glfw::Viewer viewer;
  // set_pc(viewer);
  

  Eigen::MatrixXd Dist = compute_distance_matrix(V, 12, 2, usePTU);

  MatrixXd C;
	// Assign per-vertex colors
  MatrixXd colors = Dist.row(0).transpose();
	igl::jet(colors,true,C);
  viewer.callback_key_down = &key_down; // for dealing with keyboard events
  viewer.data().add_points(V,C);

  std::cout << "Computing the Gramm matrix...";
  MatrixXd G = compute_gramm_matrix(Dist);
  std::cout << " Done !" << std::endl;

  std::cout << "Computing the new embedding...";
  std::cout.flush();
  MatrixXd Z = compute_new_embedding(G, 2);
  std::cout << " Done !" << std::endl;

  std::ofstream myfile;
  myfile.open("../data/results.txt");
  for (int i = 0; i < Z.rows(); i++) {
      myfile << Z.row(i);
      myfile << "\n";
  }
  myfile.close();

  viewer.launch();
}
