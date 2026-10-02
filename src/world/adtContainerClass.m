function m = adtContainerClass(classId)
%ADTCONTAINERCLASS  Map an AdaptaDrive class onto a drivingScenario actor.
%
%   M = ADTCONTAINERCLASS(classId) returns
%       m.kind      'vehicle' or 'actor'  (which constructor to call)
%       m.classId   the ClassID that constructor accepts
%       m.note      why this proxy was chosen
%
%   WHY A MAPPING IS NEEDED
%   drivingScenario constrains what may be created (verified on R2026a):
%       actor()    ClassID must be 3 Bicycle, 4 Pedestrian,
%                  5 Jersey Barrier or 6 Guardrail
%       vehicle()  is the constructor for 1 Car and 2 Truck
%   AdaptaDrive's own taxonomy has seven classes including auto-rickshaw,
%   pushcart and cow, none of which exist in that list.  See DEVIATIONS D11.
%
%   WHAT THIS MAPPING IS AND IS NOT
%   It affects ONLY the container's actor geometry, which is what the toolbox
%   sensor models ray-trace against.  It is NOT the class AdaptaDrive reasons
%   with: classPriors class IDs 1-7 stay authoritative through tracking, risk
%   weighting, prediction and the UI.  A cow is never treated as a pedestrian
%   anywhere a decision is made - only the sensor model's geometry proxy is
%   borrowed, and actual dimensions are always passed explicitly, so the
%   proxy never changes the size of what the sensor sees.
%
%   See also CLASSPRIORS, BUILDSCENARIO.

switch classId
    case 1   % car
        m = entry('vehicle', 1, 'Car maps directly.');
    case 2   % bus
        m = entry('vehicle', 2, ...
            'No bus class exists; Truck is the closest large vehicle proxy.');
    case 3   % auto-rickshaw
        m = entry('vehicle', 1, ...
            'No three-wheeler class exists; Car is the closest motorised proxy.');
    case 4   % two-wheeler
        m = entry('actor', 3, 'Bicycle is the closest two-wheeled proxy.');
    case 5   % pedestrian
        m = entry('actor', 4, 'Pedestrian maps directly.');
    case 6   % pushcart
        m = entry('actor', 3, ...
            'No cart class exists; Bicycle is the closest slow narrow non-motorised proxy.');
    case 7   % cow
        m = entry('actor', 4, ...
            'No animal class exists; Pedestrian is the closest unprotected mobile proxy.');
    otherwise
        m = entry('actor', 4, 'Unknown class defaults to the most cautious proxy.');
end
end

function m = entry(kind, classId, note)
m = struct('kind', kind, 'classId', classId, 'note', note);
end
