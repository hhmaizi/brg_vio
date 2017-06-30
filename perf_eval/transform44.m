function [ T ] = transform44( p, q, HorJPL )
% input: time - Time stamp
%         p    - 3by1 Translation [px py pz]'
%         q    - 4by1 Rotation quaternion
%    HorJPL    - is 0 if in Hamilton's notation and is 1 if in JPL notation 
% output: 
%         T in SE(3) a 4by4 homogeneous transformation matrix

% quat2dcm assumes Hamilton's notation
if (HorJPL == 1)
    q = [-q(4); -q(1); -q(2); -q(3)];
end

T = [quat2dcm(q') p';
    zeros(1,3) 1];

end

