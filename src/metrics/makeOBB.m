function o = makeOBB(x, y, yaw, L, W)
%MAKEOBB  Oriented bounding box: centre (x,y), heading yaw (rad), size L x W.
%
%   The box is centred on (x,y).  For the ego vehicle the reference point is
%   the REAR AXLE, so callers must shift by the axle-to-centre offset before
%   building the box - see KinematicBicycle.footprint.
%
%   Struct form (not a class) because these are created thousands of times
%   per run inside the metric loops.

o = struct('x', x, 'y', y, 'yaw', yaw, 'L', L, 'W', W);
end
