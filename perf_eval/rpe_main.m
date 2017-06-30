%% main function to compute the RPE of an estimated trajectory

%% READ ESTIMATED AND GROUND TRUTH TRAJECTORY
% Input file names
filename_gt = input('Enter name of ground truth file: ');
filename_vio = input('Enter name of estimated pose file: ');
gt_HorJPL = input('Enter 0 if quaternion for ground truth is in Hamilton notation and 1 if it is in JPL notation: ');
vio_HorJPL = input('Enter 0 if quaternion for state estimate is in Hamilton notation and 1 if it is in JPL notation: ');

% Read files
gt_matrix = dlmread(filename_gt,'\t');
vio_matrix = dlmread(filename_vio,'\t');

% Partition ground truth into time, position, orientation matrices
gt_time = gt_matrix(:,1);
gt_position = gt_matrix(:,2:4);
gt_orientation = gt_matrix(:,5:8);

% Partition state estimate into time, position, orientation matrices
vio_time = vio_matrix(:,1);
vio_position = vio_matrix(:,2:4);
vio_orientation = vio_matrix(:,5:8);

% Syncronize ground truth and estimated state
gt_sync_position = zeros(length(vio_time),3);
gt_sync_orientation = zeros(length(vio_time),4);
for i = 1:length(vio_time)
    [ index ] = find_closest_index( gt_time, vio_time(i));
    
    % Linear interpolation
    if gt_time(index) > vio_time(i)
        gt_sync_position(i,:) = gt_position(index-1,:) + (gt_position(index,:)- ...
                        gt_position(index-1,:))*((vio_time(i)- ...
                        gt_time(index-1))/(gt_time(index)-gt_time(index-1)));
        gt_sync_orientation(i,:) = gt_orientation(index-1,:) + (gt_orientation(index,:)- ...
                        gt_orientation(index-1,:))*((vio_time(i)- ...
                        gt_time(index-1))/(gt_time(index)-gt_time(index-1)));  
        gt_sync_orientation(i,:) = gt_sync_orientation(i,:)/norm(gt_sync_orientation(i,:));
    elseif (gt_time(index) < vio_time(i) && index+1 <= length(gt_time))
        gt_sync_position(i,:) = gt_position(index,:) + (gt_position(index+1,:)- ...
                        gt_position(index,:))*((vio_time(i)-gt_time(index))/...
                        (gt_time(index+1)-gt_time(index)));
        gt_sync_orientation(i,:) = gt_orientation(index,:) + (gt_orientation(index+1,:)- ...
                        gt_orientation(index,:))*((vio_time(i)-gt_time(index))/...
                        (gt_time(index+1)-gt_time(index)));
        gt_sync_orientation(i,:) = gt_sync_orientation(i,:)/norm(gt_sync_orientation(i,:));
    else
        gt_sync_position(i:end,:) = [];
        gt_sync_orientation(i:end,:) = [];
        vio_position(i:end,:) = [];
        vio_orientation(i:end,:) = [];
        vio_time(i:end,:) = [];
        break;
    end
end
gt_sync_time = vio_time;

% Set up time intervals so that we have a mimimum of 20 measurements and so
% that we can compute 100 different RPEs
mincount = 20;
dt_vec = linspace(0.1,(vio_time(end) - vio_time(1))-mincount,100);
quant_25 = zeros(1,100);
quant_med = zeros(1,100);
quant_75 = zeros(1,100);
%% Compute RPE for 100 different time spans
for f = 1:1:100
    %% Compute SE(3) transformation matrixces of the reduced state estimates
    Traj_vio = zeros(4,4,length(vio_time));
    for i = 1:length(vio_time)
        Traj_vio(:,:,i) = transform44(vio_position(i,:), vio_orientation(i,:)',vio_HorJPL);
    end
    
    %% Compute SE(3) transformation matrixces of the reduced ground truth
    Traj_gt = zeros(4,4, length(gt_sync_time));
    for i = 1:length(gt_sync_time)
        Traj_gt(:,:,i) = transform44(gt_sync_position(i,:), gt_sync_orientation(i,:)',gt_HorJPL);
    end

    %% Evaluation
    [ rpe_eval ] = evaluate_trajectory( Traj_gt, Traj_vio, vio_time, dt_vec(f), mincount );
    
    % Compute and save quantiles of translation data
    quant_25(f) = quantile(rpe_eval(2,:),0.25);
    quant_med(f)  = median(rpe_eval(2,:)); 
    quant_75(f) = quantile(rpe_eval(2,:),0.75);
end

%% Plotting
figure(1)
plot(dt_vec,quant_25,'--g',dt_vec,quant_med,'-b',dt_vec,quant_75,'--r')
xlabel('RPE Time Span (sec)')
ylabel('RPE (m)')
title('rpe: translation over many time spans')
grid on;

%% Display result for a single chosen time span
% figure(2);
% plot(rpe_eval(1,:) - rpe_eval(1,1), rpe_eval(2,:));
% xlabel('Time (sec)')
% ylabel('Position Error Drift per Second (m/s)')
% title('rpe: translation')
% grid on;
% 
% figure(3);
% plot(rpe_eval(1,:) - rpe_eval(1,1), rpe_eval(3,:));
% xlabel('Time (sec)')
% ylabel('Rotation Error (rad)')
% title('rpe: rotation')
% grid on;

%% Display Root Mean Square RPE
%RPE_RMS = rms(rpe_eval(2,:));
%display(RPE_RMS)
