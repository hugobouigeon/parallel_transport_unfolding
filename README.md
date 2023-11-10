# Parallel Transport Unfolding
Parallel Transport Unfolding is an effective method to represent higher dimensional data in lower dimension by preserving local features. It results in what coud be considered an unfolded version of the higher dimensional data. Details on our implementation and results can be found in the project report pdf.

## How to use

Make sure to have access to the Libigl library. The mesh to be used as input can be changed in the main function andshould be provided as an .off file. Results then appear in the data/results.txt file, and can be visualized using the visualize.py script.

## Visuals
Here are some visuals for our results. The left is the original 3D dataset and the right is the result obtained using our implementation of PTU.

<img src="Visuals/petit_swiss_roll/petit_swissroll_3D.png" width="40%" height="40%"> <img src="Visuals/petit_swiss_roll/petit_swiss_roll_PTU_eps_6.0.png" width="50%" height="50%">
<img src="Visuals/flower_with_base/flower_with_base_3D.png" width="40%" height="40%"> <img src="Visuals/flower_with_base/flower_with_base_PTU_knn.png" width="50%" height="50%">
<img src="Visuals/Earth_2/earth_2_3D_2.png" width="40%" height="40%"> <img src="Visuals/Earth_2/earth_2_PTU_knn.png" width="50%" height="50%">

## Contributors

Hugo Bouigeon

Paul Woringer https://github.com/paulwrg