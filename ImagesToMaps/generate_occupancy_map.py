import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import cv2
import os
file_path = "/Users/suraj/Library/CloudStorage/OneDrive-PlakshaUniversity/AgriBot/agribot/datasets/aerial_analysis/odm_filterpoints/point_cloud.ply"
if not os.path.exists(file_path):
    print(f"Error: File not found at {file_path}")
    exit()


# Load point cloud
pcd = o3d.io.read_point_cloud("/Users/suraj/Library/CloudStorage/OneDrive-PlakshaUniversity/AgriBot/agribot/datasets/aerial_analysis/odm_filterpoints/point_cloud.ply")

# Segment the ground plane using RANSAC
# Parameters: distance_threshold (points within this distance to the plane are considered inliers),
#             ransac_n (number of random points to sample for plane fitting),
#             num_iterations (number of RANSAC iterations)
distance_threshold = 0.06  # Adjust this based on your point cloud density and ground flatness
ransac_n = 3
num_iterations = 1000

plane_model, inliers = pcd.segment_plane(distance_threshold, ransac_n, num_iterations)
ground_pcd = pcd.select_by_index(inliers)
non_ground_pcd = pcd.select_by_index(inliers, invert=True)

# Now, use non_ground_pcd for creating your heightmap
points = np.asarray(non_ground_pcd.points)


# Create grid
resolution = 0.05  # meters per pixel
min_x, min_y = np.min(points[:, :2], axis=0)
max_x, max_y = np.max(points[:, :2], axis=0)
width = int((max_x - min_x) / resolution)
height = int((max_y - min_y) / resolution)

print(f"Calculated width: {width}, height: {height}")

# Initialize heightmap
heightmap = np.full((height, width), -np.inf)

# Fill heightmap
for x, y, z in points:
    ix = int((x - min_x) / resolution)
    iy = int((y - min_y) / resolution)

    # Clamp indices to ensure they are within bounds
    ix = np.clip(ix, 0, width - 1)
    iy = np.clip(iy, 0, height - 1)

    heightmap[iy, ix] = max(heightmap[iy, ix], z)


# Normalize and threshold for occupancy
heightmap[heightmap == -np.inf] = np.nan
occupancy = np.where(np.isnan(heightmap), 255, 0).astype(np.uint8)  # 255 = unknown, 0 = free

# Save as PGM
cv2.imwrite("map2.pgm", occupancy)

# Create YAML
with open("map2.yaml", "w") as f:
    f.write(f"""image: map2.pgm
resolution: {resolution}
origin: [{min_x}, {min_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.2
""")
