from open3d import *
import numpy as np 
import csv
import os
import sys

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
    
# data directory
raw_pcd_dir = '/media/gskim/GISEOP2TB/paper/IROS2019-2/20180911 pointnet cls with 프리드버그 campus 77 class 실습/data/freiburgCampus360_3D/'

# parameters 
num_constant_points = 7000
pcd_save_dir = '/media/gskim/GISEOP2TB/paper/IROS2019-2/20180911 pointnet cls with 프리드버그 campus 77 class 실습/data/freiburgCampus-' + str(num_constant_points) + '/'

downsample_voxel_size = 0.3


# for each 3D ptcloud 
num_ptclouds_in_dataset = 77
for i in range(num_ptclouds_in_dataset):
            
    # 0. load 
    scan_num = str(i+1).zfill(3)
    pcd_base_name = 'scan_' + scan_num + '_points'
    raw_pcd_path = raw_pcd_dir + 'scan_' + scan_num + '_points.dat'

    content_all, ptcloud = read_dat(raw_pcd_path)
    

    # 1. save raw 
    raw_pcd = PointCloud()
    raw_pcd.points = Vector3dVector(ptcloud)

    raw_pcd_ply_save_path = pcd_save_dir + 'ply/raw/' + pcd_base_name + '.ply' 
    raw_pcd_npy_save_path = pcd_save_dir + 'npy/raw/' + pcd_base_name + '.npy' 

    write_point_cloud(raw_pcd_ply_save_path, raw_pcd)
    np.save(raw_pcd_npy_save_path, np.asarray(raw_pcd.points))


    # 2. save downsampled
    down_pcd = voxel_down_sample(raw_pcd, voxel_size = downsample_voxel_size)
    
    down_pcd_name = pcd_base_name + '_down'
    down_pcd_ply_save_path = pcd_save_dir + 'ply/down/' + down_pcd_name + '.ply' 
    down_pcd_npy_save_path = pcd_save_dir + 'npy/down/' + down_pcd_name + '.npy' 
    
    write_point_cloud(down_pcd_ply_save_path, down_pcd)
    np.save(down_pcd_npy_save_path, np.asarray(down_pcd.points))

    
    # 3. save road-removed 
    estimate_normals(raw_pcd, search_param = KDTreeSearchParamHybrid(radius = 1, max_nn = 30)) # empirically, radius 1 was good for outdoor, noisy, and sparse point cloud.
    
    raw_points = np.asarray(raw_pcd.points)
    num_raw_points = len(raw_points)

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
    
    road_removed_raw_pcd = PointCloud()
    road_removed_raw_pcd.points = Vector3dVector(road_removed_points)

    road_removed_raw_pcd_name = pcd_base_name + '_road_removed'
    road_removed_raw_pcd_ply_save_path = pcd_save_dir + 'ply/road_removed/' + road_removed_raw_pcd_name + '.ply' 
    road_removed_raw_pcd_npy_save_path = pcd_save_dir + 'npy/road_removed/' + road_removed_raw_pcd_name + '.npy' 

    write_point_cloud(road_removed_raw_pcd_ply_save_path, road_removed_raw_pcd)
    np.save(road_removed_raw_pcd_npy_save_path, np.asarray(road_removed_raw_pcd.points))


    # 4. save downsampled road-removed
    road_removed_down_pcd = voxel_down_sample(road_removed_raw_pcd, voxel_size = downsample_voxel_size)

    road_removed_down_pcd_name = pcd_base_name + '_road_removed_down'
    road_removed_down_pcd_ply_save_path = pcd_save_dir + 'ply/road_removed_down/' + road_removed_down_pcd_name + '.ply' 
    road_removed_down_pcd_npy_save_path = pcd_save_dir + 'npy/road_removed_down/' + road_removed_down_pcd_name + '.npy' 
    
    write_point_cloud(road_removed_down_pcd_ply_save_path, road_removed_down_pcd)
    np.save(road_removed_down_pcd_npy_save_path, np.asarray(road_removed_down_pcd.points))

    road_removed_down_points = np.asarray(road_removed_down_pcd.points)
    num_road_removed_down_points = len(road_removed_down_points)
    print(num_road_removed_down_points)
    
    
    # 5. save const-num road-removed 
    if(num_constant_points > num_road_removed_down_points):
        print('There are not enough points!')
        sys.exit(1)

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

    road_removed_const_num_points = np.array(road_removed_const_num_points)
    
    road_removed_const_num_pcd = PointCloud()
    road_removed_const_num_pcd.points = Vector3dVector(road_removed_const_num_points)

    road_removed_const_num_pcd_name = pcd_base_name + '_road_removed_const_num' + str(num_constant_points)
    road_removed_const_num_pcd_ply_save_path = pcd_save_dir + 'ply/road_removed_const_num/' + road_removed_const_num_pcd_name + '.ply' 
    road_removed_const_num_pcd_npy_save_path = pcd_save_dir + 'npy/road_removed_const_num/' + road_removed_const_num_pcd_name + '.npy' 

    write_point_cloud(road_removed_const_num_pcd_ply_save_path, road_removed_const_num_pcd)
    np.save(road_removed_const_num_pcd_npy_save_path, np.asarray(road_removed_const_num_pcd.points))


    # 6. save scaled const-num road-removed 
    # TBA

    
    
    
    
    
    
    
    
    

    
