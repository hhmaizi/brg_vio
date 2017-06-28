%% main function to compute the RPE of an estimated trajectory

%% READ ESTIMATED AND GROUND TRUTH TRAJECTORY
% Input file names
filename_gt = input('Enter name of ground truth file: ');
filename_vio = input('Enter name of estimated pose file: ');

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
dt_vec = linspace(0.1,(vio_time(end) - vio_time(1))/20,100);
quant_25 = zeros(1,100);
quant_med = zeros(1,100);
quant_75 = zeros(1,100);
%% Compute RPE for 100 different time spans
for f = 1:1:100
    %% Compute RPE assuming that we need a minimum of 20 measurements
    dt = dt_vec(f);
    % Compute the number of RPE calculations for time difference dt
    intervals = floor((vio_time(end) - vio_time(1))/dt);
    % Define a matrix of estimated state at dt time intervals
    reducedvio_time = zeros(intervals,1);
    reducedvio_position = zeros(3,intervals);
    reducedvio_orientation = zeros(4,intervals);
    % Define the initial state estimate
    reducedvio_time(1) = vio_time(1);
    reducedvio_position(:,1) = vio_position(1,:)';
    reducedvio_orientation(:,1) = vio_orientation(1,:)';
    % Define the rest of the reduced estimated state matrices
    j = 1;
    for i = 1:length(vio_time)
        if (vio_time(i) - reducedvio_time(j)) >= dt
            j = j + 1;
            reducedvio_time(j) = vio_time(i);
            reducedvio_position(:,j) = vio_position(i,:)';
            reducedvio_orientation(:,j) = vio_orientation(i,:)';
        end
    end
    reducedvio_time(j+1:end) = [];
    reducedvio_position(:,j+1:end) = [];
    reducedvio_orientation(:,j+1:end) = [];
    
    %% Compute SE(3) transformation matrixces of the reduced state estimates
    Traj_vio = zeros(4,4,length(reducedvio_time));
    for i = 1:length(reducedvio_time)
        Traj_vio(:,:,i) = transform44(reducedvio_position(:,i), reducedvio_orientation(:,i));
    end
    
    %% Define a matrix of ground truth at dt time intervals
    reducedgt_time = zeros(intervals,1);
    reducedgt_position = zeros(3,intervals);
    reducedgt_orientation = zeros(4,intervals);
    % Define the initial state estimate
    reducedgt_time(1) = gt_sync_time(1);
    reducedgt_position(:,1) = gt_sync_position(1,:)';
    reducedgt_orientation(:,1) = gt_sync_orientation(1,:)';
    % Define the rest of the reduced estimated state matrices
    j = 1;
    for i = 1:length(gt_sync_time)
        if (gt_sync_time(i) - reducedgt_time(j)) >= dt
            j = j + 1;
            reducedgt_time(j) = gt_sync_time(i);
            reducedgt_position(:,j) = gt_sync_position(i,:)';
            reducedgt_orientation(:,j) = gt_sync_orientation(i,:)';
        end
    end
    reducedgt_time(j+1:end) = [];
    reducedgt_position(:,j+1:end) = [];
    reducedgt_orientation(:,j+1:end) = [];
    
    %% Compute SE(3) transformation matrixces of the reduced ground truth
    Traj_gt = zeros(4,4, length(reducedgt_time));
    for i = 1:length(reducedgt_time)
        Traj_gt(:,:,i) = transform44(reducedgt_position(:,i), reducedgt_orientation(:,i));
    end
    
    %% Evaluation
    [ rpe_eval ] = evaluate_trajectory( Traj_gt, Traj_vio, reducedvio_time );
    
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
