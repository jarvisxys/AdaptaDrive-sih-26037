function d = pointOBBDistance(px, py, o)
%POINTOBBDISTANCE  Distance from points to an oriented bounding box.
%
%   D = POINTOBBDISTANCE(PX, PY, O) is 0 for points inside the box and the
%   shortest distance to its boundary otherwise.  Vectorised over PX/PY.
%
%   See also OBBDISTANCE, POINTINOBB.

c = cos(o.yaw);
s = sin(o.yaw);
rx =  (px - o.x) * c + (py - o.y) * s;    % into the box frame
ry = -(px - o.x) * s + (py - o.y) * c;

dx = max(abs(rx) - o.L/2, 0);
dy = max(abs(ry) - o.W/2, 0);
d = hypot(dx, dy);
end
