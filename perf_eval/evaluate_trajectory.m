function [ rpe_eval ] = evaluate_trajectory( traj_gt, traj_est, time_est )
% Input:
%           - traj_gt: 4by4byn matrix the ground truth trajectory
%           - time_gt: time stamp of ground truth trajectory
%           - traj_est: 4by4bym matrix the estimated trajectory
%           - time_est: time stamp of state estimate trajectory
% Output:
%           - time_align: the ground truth time which is closest to
%           estimated time traj
%           - idx_gt_align: the index of the aligned time from ground truth
%           - rpe_eval: 3bym matrix row 1 is the time_est, row 2 is the
%           translational error, row 3 is the rotational error.
    
rpe_eval = zeros(3,length(time_est)-1);
for i = 1:length(time_est)-1
    % Compute relative pose error at time step i
    error44 = invSE3(invSE3(traj_est(:,:,i))*traj_est(:,:,i+1))*(invSE3(traj_gt(:,:,i))*traj_gt(:,:,i+1));
    % Compute norm of translation error
    trans = norm(error44(1:3,4));
    % Compute error in rotation as an angle
    rotation = compute_angle(error44);
    % Save RPE errors
    rpe_eval(1,i) = time_est(i);
    rpe_eval(2,i) = trans;
    rpe_eval(3,i) = rotation;
end

end

% Compute inverse of an SE3 matrix
function out = invSE3(in)
    out = [in(1:3,1:3)', -in(1:3,1:3)'*in(1:3,4);in(4,:)];
end
