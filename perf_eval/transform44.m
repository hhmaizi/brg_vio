function [ T ] = transform44( p, q )
% input: time - Time stamp
%         p    - 3by1 Translation [px py pz]'
%         q    - 4by1 Rotation quaternion [qx qy qz qw]'
% output: 
%         T in SE(3) a 4by4 homogeneous transformation matrix

% quat2dcm assumes scalar in quaternion goes first. We have assumed that it
% goes last.
q = [q(2); q(3); q(4); q(1)];


T = [quat2dcm(q') p';
    zeros(1,3) 1];

end

