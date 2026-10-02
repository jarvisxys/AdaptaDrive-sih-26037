function [fill, edge] = adClassColor(classId)
%ADCLASSCOLOR  Fill and outline colour for an agent class.
%
%   classId 1..7 as fixed in CLASSPRIORS; classId 0 (unknown) maps to the
%   grey prototype colour.  Single lookup so the bird's-eye view, the legend
%   and the results charts cannot drift apart.

t = theme();
if classId >= 1 && classId <= 7
    row = classId;
else
    row = 8;   % unknown
end
fill = t.classFill(row, :);
edge = t.classEdge(row, :);
end
