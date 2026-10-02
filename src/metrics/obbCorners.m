function C = obbCorners(o)
%OBBCORNERS  The four corners of an oriented bounding box, counter-clockwise.
%
%   C = OBBCORNERS(o) returns a 4x2 matrix of [x y] corners, ordered
%   front-left, rear-left, rear-right, front-right.  That traversal is
%   counter-clockwise, which the separating-axis and distance routines rely
%   on.
%
%   See also MAKEOBB, OBBOVERLAP, OBBDISTANCE.

hl = o.L / 2;
hw = o.W / 2;
c = cos(o.yaw);
s = sin(o.yaw);

% Body frame: +x forward, +y left.
local = [ hl,  hw
         -hl,  hw
         -hl, -hw
          hl, -hw];

R = [c, -s; s, c];
C = local * R.' + [o.x, o.y];
end
