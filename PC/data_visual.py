import numpy as np
import open3d as o3d

print("Read in the prism point cloud data (pcd)")
pcd = o3d.io.read_point_cloud("data_final.xyz", format="xyz")

# Lets see what our point cloud data looks like numerically
print("The PCD array:")
#np.asarray converts the input into an array
print(np.asarray(pcd.points))


# Lets see what our point cloud data looks like graphically
print("Lets visualize the PCD: (spawns seperate interactive window)")
o3d.visualization.draw_geometries([pcd])

# OK, good, but not great, lets add some lines to connect the vertices
#   For creating a lineset we will need to tell the package which vertices need connected
#   Remember each vertex actually contains one x,y,z coordinate
count=10;
# Give each vertex a unique number
total=32*count
yz_slice_vertex = []
for x in range(0, total):
    yz_slice_vertex.append([x])



# Define coordinates to connect lines in each yz slice
lines = []

for x in range(0, total, 32):
    lines.append([yz_slice_vertex[x], yz_slice_vertex[x + 1]])
    lines.append([yz_slice_vertex[x + 1], yz_slice_vertex[x + 2]])
    lines.append([yz_slice_vertex[x + 2], yz_slice_vertex[x + 3]])
    lines.append([yz_slice_vertex[x + 3], yz_slice_vertex[x + 4]])
    lines.append([yz_slice_vertex[x + 4], yz_slice_vertex[x + 5]])
    lines.append([yz_slice_vertex[x + 5], yz_slice_vertex[x + 6]])
    lines.append([yz_slice_vertex[x + 6], yz_slice_vertex[x + 7]])
    lines.append([yz_slice_vertex[x + 7], yz_slice_vertex[x + 8]])
    lines.append([yz_slice_vertex[x + 8], yz_slice_vertex[x + 9]])
    lines.append([yz_slice_vertex[x + 9], yz_slice_vertex[x + 10]])
    lines.append([yz_slice_vertex[x + 10], yz_slice_vertex[x + 11]])
    lines.append([yz_slice_vertex[x + 11], yz_slice_vertex[x + 12]])
    lines.append([yz_slice_vertex[x + 12], yz_slice_vertex[x + 13]])
    lines.append([yz_slice_vertex[x + 13], yz_slice_vertex[x + 14]])
    lines.append([yz_slice_vertex[x + 14], yz_slice_vertex[x + 15]])
    lines.append([yz_slice_vertex[x + 15], yz_slice_vertex[x + 16]])
    lines.append([yz_slice_vertex[x + 16], yz_slice_vertex[x + 17]])
    lines.append([yz_slice_vertex[x + 17], yz_slice_vertex[x + 18]])
    lines.append([yz_slice_vertex[x + 18], yz_slice_vertex[x + 19]])
    lines.append([yz_slice_vertex[x + 19], yz_slice_vertex[x + 20]])
    lines.append([yz_slice_vertex[x + 20], yz_slice_vertex[x + 21]])
    lines.append([yz_slice_vertex[x + 21], yz_slice_vertex[x + 22]])
    lines.append([yz_slice_vertex[x + 22], yz_slice_vertex[x + 23]])
    lines.append([yz_slice_vertex[x + 23], yz_slice_vertex[x + 24]])
    lines.append([yz_slice_vertex[x + 24], yz_slice_vertex[x + 25]])
    lines.append([yz_slice_vertex[x + 25], yz_slice_vertex[x + 26]])
    lines.append([yz_slice_vertex[x + 26], yz_slice_vertex[x + 27]])
    lines.append([yz_slice_vertex[x + 27], yz_slice_vertex[x + 28]])
    lines.append([yz_slice_vertex[x + 28], yz_slice_vertex[x + 29]])
    lines.append([yz_slice_vertex[x + 29], yz_slice_vertex[x + 30]])
    lines.append([yz_slice_vertex[x + 30], yz_slice_vertex[x + 31]])
    lines.append([yz_slice_vertex[x + 31], yz_slice_vertex[x]])

# Define coordinates to connect lines between current and next yz slice
# we are now connecting the faces, total-32 vertices
for x in range(0, total-32, 32):
    lines.append([yz_slice_vertex[x], yz_slice_vertex[x + 32]])
    lines.append([yz_slice_vertex[x + 1], yz_slice_vertex[x + 33]])
    lines.append([yz_slice_vertex[x + 2], yz_slice_vertex[x + 34]])
    lines.append([yz_slice_vertex[x + 3], yz_slice_vertex[x + 35]])
    lines.append([yz_slice_vertex[x+ 4], yz_slice_vertex[x + 36]])
    lines.append([yz_slice_vertex[x + 5], yz_slice_vertex[x + 37]])
    lines.append([yz_slice_vertex[x + 6], yz_slice_vertex[x + 38]])
    lines.append([yz_slice_vertex[x + 7], yz_slice_vertex[x + 39]])
    lines.append([yz_slice_vertex[x+ 8], yz_slice_vertex[x + 40]])
    lines.append([yz_slice_vertex[x + 9], yz_slice_vertex[x + 41]])
    lines.append([yz_slice_vertex[x + 10], yz_slice_vertex[x + 42]])
    lines.append([yz_slice_vertex[x + 11], yz_slice_vertex[x + 43]])
    lines.append([yz_slice_vertex[x+ 12], yz_slice_vertex[x + 44]])
    lines.append([yz_slice_vertex[x + 13], yz_slice_vertex[x + 45]])
    lines.append([yz_slice_vertex[x + 14], yz_slice_vertex[x + 46]])
    lines.append([yz_slice_vertex[x + 15], yz_slice_vertex[x + 47]])
    lines.append([yz_slice_vertex[x+ 16], yz_slice_vertex[x + 48]])
    lines.append([yz_slice_vertex[x + 17], yz_slice_vertex[x + 49]])
    lines.append([yz_slice_vertex[x + 18], yz_slice_vertex[x + 50]])
    lines.append([yz_slice_vertex[x+ 19], yz_slice_vertex[x + 51]])
    lines.append([yz_slice_vertex[x + 20], yz_slice_vertex[x + 52]])
    lines.append([yz_slice_vertex[x + 21], yz_slice_vertex[x + 53]])
    lines.append([yz_slice_vertex[x + 22], yz_slice_vertex[x + 54]])
    lines.append([yz_slice_vertex[x+ 23], yz_slice_vertex[x + 55]])
    lines.append([yz_slice_vertex[x + 24], yz_slice_vertex[x + 56]])
    lines.append([yz_slice_vertex[x + 25], yz_slice_vertex[x + 57]])
    lines.append([yz_slice_vertex[x + 26], yz_slice_vertex[x + 58]])
    lines.append([yz_slice_vertex[x+ 27], yz_slice_vertex[x + 59]])
    lines.append([yz_slice_vertex[x + 28], yz_slice_vertex[x + 60]])
    lines.append([yz_slice_vertex[x + 29], yz_slice_vertex[x + 61]])
    lines.append([yz_slice_vertex[x + 30], yz_slice_vertex[x + 62]])
    lines.append([yz_slice_vertex[x+ 31], yz_slice_vertex[x + 63]])



 # This line maps the lines to the 3d coordinate vertices
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),
                                    lines=o3d.utility.Vector2iVector(lines))

# Lets see what our point cloud data with lines looks like graphically
o3d.visualization.draw_geometries([line_set])



