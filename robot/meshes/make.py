import open3d as o3d

# Load OBJ file
mesh = o3d.io.read_triangle_mesh("noodle.obj")

# Convert to PointCloud and save as PCD file
pcd = mesh.sample_points_uniformly(number_of_points=100000)
o3d.io.write_point_cloud("cup.pcd", pcd)
