close all;
clear all;
%%gyro measurement = 3x1 column vector
%%R measurement = quaternion form 

gyro_samples =200;   %%high frequency measurement
R_samples = 50;      %%lo frequency measurement
ang_vel = [0;1;0];  %constant omega/rad

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%generate data with gaussian noise
sigma_noise_lo = 0.002;
sigma_noise_hi = 0.02;
R_ini = eye(3);

ang_vel_buffer = ones(3,gyro_samples).*ang_vel;
ang_vel_skew_buf = [];
rot_noisy =[];
rot_quat_ini = rotm2quat(R_ini);
rotation_quat_noisy =[rot_quat_ini];

noise_hi = sigma_noise_hi * randn(3,gyro_samples);
noisy_ang_vel = ang_vel_buffer + noise_hi;

noise_lo = sigma_noise_lo * randn(3,gyro_samples);
noisy_ang_vel_lo = ang_vel_buffer + noise_lo;

for i =1:gyro_samples
    ang_vel_skew_noisy = func_skew(noisy_ang_vel(:,1));
    %ang_vel_skew_buf = [ang_vel_skew_buf;ang_vel_skew_noisy];
    R_noisy = R_ini*expm(ang_vel_skew_noisy);
    rot_noisy = [rot_noisy;R_noisy];
    quat_noisy = rotm2quat(R_noisy);
    rotation_quat_noisy = [rotation_quat_noisy;quat_noisy];
    R_ini = R_noisy;
    
end

rot_lo_measured_noisy = rotation_quat_noisy(1:gyro_samples/R_samples:end,:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%string to differentiate types of measurement
str_edge = "EDGE_SO3:VECTOR";  
str_vertex = "VERTEX_SO3:QUAT";

%%func_skew to convert 3x1 vector to skew symmetric matrix
ang_vel_skew = func_skew(ang_vel);
%ang_vel_skew =[ 0 -ang_vel(3) ang_vel(2); ang_vel(3) 0 -ang_vel(1); -ang_vel(2) ang_vel(1) 0];

R_prev = eye(3); %Initialize with identity matrix
quat_ini = rotm2quat(R_prev);
rotation = []; %contains R(0),R(1),R(k)
rotation_quat = [quat_ini];

% constant input
for k =1:gyro_samples
    R = R_prev*expm(ang_vel_skew);
    rotation = [rotation;R];
    quat = rotm2quat(R);
    rotation_quat = [rotation_quat;quat];
    R_prev = R;
    
end

rot_lo_measured = rotation_quat(1:gyro_samples/R_samples:end,:);




%% write data to txt file
fname = 'omega.txt'
fid = fopen(fname,'w');
if fid ~= -1
    for i=1:R_samples
        fprintf(fid,'%s %d %d %d %d\n',str_vertex+" "+(i-1), rot_lo_measured(i,:));
    end
    fclose (fid);
end

%%
fid = fopen(fname,'a');
if fid ~= -1
    for i = 1:gyro_samples
        fprintf(fid,'%s %d %d %d\n', str_edge,[ang_vel_buffer(:,i)]');
    end
end

fname = 'ground_truth.txt'
fid = fopen(fname,'w');
if fid ~= -1
    for i=1:R_samples
        fprintf(fid,'%d %d %d %d\n', rot_lo_measured(i,:));
    end
    fclose (fid);
end




%%
file = 'omega_noisy.txt'
fid_noisy = fopen(file,'w');

if fid_noisy ~= -1
    for i=1:R_samples
        fprintf(fid_noisy,'%s %d %d %d %d\n', str_vertex+" "+(i-1), rot_lo_measured_noisy(i,:));
    end
    fclose(fid_noisy);
end

fid_noisy = fopen(file,'a');
if fid_noisy ~= -1
    for i=1:gyro_samples
        fprintf(fid_noisy,'%s %d %d %d\n', str_edge, [noisy_ang_vel_lo(:,i)]');
    end
end
fclose(fid_noisy);


%dlmwrite(fname,ang_vel_buffer','-append','delimiter',' ');
%dlmwrite('Rotation_input.txt',A);
%dlmwrite('Rotation_input.txt',rotation);
%csvwrite('imu_measurement.csv',[id ,ang_vel_buffer'])


