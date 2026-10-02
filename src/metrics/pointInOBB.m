function tf = pointInOBB(px, py, o)
%POINTINOBB  True for points inside (or on) an oriented bounding box.
%   Vectorised over PX/PY.

c = cos(o.yaw);
s = sin(o.yaw);
rx =  (px - o.x) * c + (py - o.y) * s;
ry = -(px - o.x) * s + (py - o.y) * c;

tf = abs(rx) <= o.L/2 & abs(ry) <= o.W/2;
end
