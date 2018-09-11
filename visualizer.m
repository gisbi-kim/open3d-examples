
data_dir = '/media/gskim/GISEOP2TB/paper/IROS2019-2/20180911 pointnet cls with 프리드버그 campus 77 class 실습/data/freiburgCampus-7000-past/';
save_dir = '/media/gskim/GISEOP2TB/paper/IROS2019-2/20180911 pointnet cls with 프리드버그 campus 77 class 실습/data/freiburgCampus-7000-captures/';


num_ptclouds = 77;
for i = 1:num_ptclouds
    
    scan_num = sprintf( '%03d', i ) ;
    disp(scan_num)
    
    % road-removed and having constant number 
    ith_ptcloud_path = [data_dir, 'scan_', scan_num, '_points_road_removed_const_num7000.ply'];
    ith_ptcloud = pcread(ith_ptcloud_path);
    
    figure(1); clf;
    pcshow(ith_ptcloud)
    disp(ith_ptcloud.Count);
    
    xlim([-60, 60]); ylim([-60, 60]);
    save_name = [save_dir, 'scan_', scan_num, '.png'];
    saveas(gcf, save_name)
 
end