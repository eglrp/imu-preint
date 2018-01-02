
rot_mat_estimated = [];
rot_mat_groundtruth = [];
%%read txt file containing quaternion
no_of_poses = 50;
file = 'poses_optimized.txt';
ground_truth = 'ground_truth.txt';

%%open file
optimized_f = fopen(file,'r');
ground_truth = fopen(ground_truth,'r');

%%read the file
q = dlmread('poses_optimized.txt');
gt = dlmread('ground_truth.txt');
theta_buffer =[];

for k=1:no_of_poses
    b = q(k,:);
    a = gt(k,:);
    rot_mat_estimated = quat2rotm(b);
    rot_mat_groundtruth = quat2rotm(a);
    trace_rot = inv(rot_mat_estimated) * rot_mat_groundtruth
    theta = acos((trace(trace_rot)-1)/2)
    theta_buffer = [theta_buffer;theta];
    %rot_mat_b = [rot_mat_b;rot];
    %rot_mat_a = [rot_mat_a;rot_a];
end

title('Robot pose in 3D');

figure;
j=1:1:50;
plot(j,theta_buffer(j),'-go');
