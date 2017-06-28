function [ angle ] = compute_angle( transform )
%       Input: transform in SE(3) 4by4 Matrix
%       Output: rotation angle from the translation scalar

% an invitation to 3d vision page 27

angle = acos(min(1,max(-1,(trace(transform(1:3,1:3)) - 1 )/2 ) ) );

end

