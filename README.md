# Planar_Polygons_Detector (PolyDetect)

PolyDetect: an efficient algorithm to extract planar polygons from LiDAR data based on MSAC (M-estimator SAmple and Consensus).

# Required dependencies:

PCL, Eigen, CGAL.

# Test:

./PolyDetect ../data/musee_RDC003.ply 0.03 1000 2000 0.06
 
where:

0.03: inliers threshold.

1000: maximum number of the iteration. 

2000: minimum size of a planar region. 

0.06: the parameter alpha (alpha shape).

(the values of these parameters can be modified and adapted by the users ).

# How it works?

# First Part: Planes detection

1. It estimates the plane parameters  using MSAC.
2. It selects the inliers of the estimated plane.
3. It uses the least squares fitting to estimate the average  plane of these inliers.
4. It removes these inliers  from an input point cloud.
5. If the size of the obtained planar region > the minimum size of a planar region, it repeats steps 1 to 4, otherwise it stops.

# Second part: Polygons extraction:
For each estimated plane:
1. It  projects the inliers on the plane.
2. It extracts the set of polygons with holes using alpha shape.


# Third part: Visualization

1. It generate a grid points. 
2. It projects the points of the generated grid onto the supporting plane plane of the polygon.
3. It keeps 3D points that project inside the polygon.


# If our project is helpful for your research, please consider citing:

Djahel, Rahima, Bruno Vallet, and Pascal Monasse. "Towards Efficient Indoor/Outdoor Registration Using Planar Polygons." ISPRS annals of the photogrammetry, remote sensing and spatial information sciences 2 (2021): 51-58.

# To contact us:

rahima.djahel@enpc.fr

rdjahel@gmail.com










