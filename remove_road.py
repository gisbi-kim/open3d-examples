
# coding: utf-8

# In[155]:


from open3d import *
import numpy as np 
import csv
import os
import sys


# In[156]:


def read_dat(file_path):

    content = []
    ptcloud = []

    file = open(file_path, 'r')
    for line in file:
        fields = line.split(' ')
        
        robot_pose_x = float(fields[0])
        robot_pose_y = float(fields[1])
        robot_pose_z = float(fields[2])
        point_x = float(fields[3])
        point_y = float(fields[4])
        point_z = float(fields[5])
        
        content_all = np.array([robot_pose_x, robot_pose_y, robot_pose_z, point_x, point_y, point_z])
        content_ptcloud = np.array([point_x, point_y, point_z])
        
        content.append(content_all)
        ptcloud.append(content_ptcloud)

    content = np.array(content)
    ptcloud = np.array(ptcloud)
    
    return content, ptcloud


# In[157]:


"""
Read Points (from dat file)
"""

raw_pcd_dir = './data/sample_data/freiburgCampus360_3D/'
scan_num = '001'

pcd_base_name = 'scan_' + scan_num + '_points'
raw_pcd_path = raw_pcd_dir + 'scan_' + scan_num + '_points.dat'

content_all, ptcloud = read_dat(raw_pcd_path)


# In[158]:


ptcloud


# In[159]:


"""
Convert numpy array into ply form (available for open3d functions)
"""
raw_pcd = PointCloud()
raw_pcd.points = Vector3dVector(ptcloud)

pcd_save_dir = './data/processed_data/freiburgCampus360_3D/'
raw_pcd_save_path = pcd_save_dir + pcd_base_name + '.ply' 

# save raw point cloud 
write_point_cloud(raw_pcd_save_path, raw_pcd)


# In[160]:


# - Load saved point cloud and visualize it
raw_pcd_load = read_point_cloud(raw_pcd_save_path)
draw_geometries([raw_pcd_load])


# In[161]:


"""
Downsampling
"""
down_pcd = voxel_down_sample(raw_pcd, voxel_size = 0.1)
draw_geometries([down_pcd])

down_pcd_name = pcd_base_name + '_down'
down_pcd_save_path = pcd_save_dir + down_pcd_name + '.ply' 

# save downsampled point cloud 
write_point_cloud(down_pcd_save_path, down_pcd)


# In[162]:


print(raw_pcd)
print(down_pcd)


# In[163]:


# - calc normals
estimate_normals(raw_pcd, search_param = KDTreeSearchParamHybrid(
        radius = 1, max_nn = 30)) # empirically, radius 1 was good for outdoor, noisy, and sparse point cloud.
draw_geometries([raw_pcd])


# In[164]:


# numpy points for processing
raw_points = np.asarray(raw_pcd.points)

num_raw_points = len(raw_points)
print(num_raw_points)


# In[165]:


"""
removing road
"""
# removing road points 
normals = np.asarray(raw_pcd.normals)

road_removed_points = []
for i in range(num_raw_points):
    ith_point_xyz = raw_points[i, :]
    ith_point_normal = normals[i, :]

    z_normal = ith_point_normal[2]
    e = 0.05
    if( np.abs( np.abs(z_normal) - 1) < e):
        pass
    else:
        road_removed_points.append(ith_point_xyz)
        
road_removed_points = np.array(road_removed_points)


# In[166]:


# save road-removed point cloud 
road_removed_raw_pcd = PointCloud()
road_removed_raw_pcd.points = Vector3dVector(road_removed_points)

# save 
road_removed_raw_pcd_name = pcd_base_name + '_road_removed'
road_removed_raw_pcd_save_path = pcd_save_dir + road_removed_raw_pcd_name + '.ply' 
write_point_cloud(road_removed_raw_pcd_save_path, road_removed_raw_pcd)


# Load saved point cloud and visualize it
road_removed_raw_pcd_load = read_point_cloud(road_removed_raw_pcd_save_path)
draw_geometries([road_removed_raw_pcd_load])

print(raw_pcd)
print(road_removed_raw_pcd_load)


# In[167]:


"""
Downsampling of the road-removed point cloud
"""
road_removed_down_pcd = voxel_down_sample(road_removed_raw_pcd, voxel_size = 0.3)
draw_geometries([road_removed_down_pcd])

# save 
road_removed_down_pcd_name = pcd_base_name + '_road_removed_down'
road_removed_down_pcd_save_path = pcd_save_dir + road_removed_down_pcd_name + '.ply' 
write_point_cloud(road_removed_down_pcd_save_path, road_removed_down_pcd)

# number of downsampled points (without road)
# print(road_removed_down_pcd)
road_removed_down_points = np.asarray(road_removed_down_pcd.points)
num_road_removed_down_points = len(road_removed_down_points)
print(num_road_removed_down_points)     


# In[168]:


"""
Filtering the point cloud into the fixed number of points 
"""
# target the number of points
num_constant_points = 7000
if(num_constant_points > num_road_removed_down_points):
    print('There are not enough points!')
    sys.exit(1)

# numpy points for processing
road_removed_down_points = np.asarray(road_removed_down_pcd.points)

num_road_removed_down_points = len(road_removed_down_points)
print(num_road_removed_down_points)


# randomly select the number of points and resave 
road_removed_const_num_points = []

already_selected_point_index = [] 
count = 0

while(1):
    rand_index = np.random.randint(num_road_removed_down_points)
    
    if(rand_index in already_selected_point_index):
        pass
    else:
        the_point = road_removed_down_points[rand_index, :]
        road_removed_const_num_points.append(the_point)

        already_selected_point_index.append(rand_index)
        count = count + 1 

    if(count==num_constant_points):
        break


# numpy format 
road_removed_const_num_points = np.array(road_removed_const_num_points)


# In[169]:


# ply format
road_removed_const_num_pcd = PointCloud()
road_removed_const_num_pcd.points = Vector3dVector(road_removed_const_num_points)

road_removed_const_num_pcd_name = pcd_base_name + '_road_removed_const_num' + str(num_constant_points)
road_removed_const_num_pcd_save_path = pcd_save_dir + road_removed_const_num_pcd_name + '.ply' 

# save 
write_point_cloud(road_removed_const_num_pcd_save_path, road_removed_const_num_pcd)

# viz 
draw_geometries([road_removed_const_num_pcd])
print(road_removed_const_num_pcd)

