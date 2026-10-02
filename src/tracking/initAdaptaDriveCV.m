function filter = initAdaptaDriveCV(detection)
%INITADAPTADRIVECV  Constant-velocity EKF initialiser for AdaptaDrive tracks.
%
%   Wraps INITCVEKF and fixes one thing it cannot know about our sensor mix:
%   the camera does not measure velocity.
%
%   THE PROBLEM
%   multiObjectTracker requires a uniform measurement size, so camera
%   detections must still carry velocity fields, declared "unobserved" with a
%   very large measurement variance (1e6).  initcvekf seeds a new track's
%   velocity covariance FROM THAT VARIANCE.  A track born on a camera
%   detection therefore starts with ~1e6 m^2/s^2 of velocity uncertainty,
%   which a single 0.1 s prediction turns into ~1e4 m^2 of position
%   uncertainty.  The association gate stops meaning anything, tracks are
%   abandoned and re-created, and one pedestrian ends up carrying four
%   simultaneous tracks.
%
%   THE FIX
%   Cap the initial velocity covariance at a physically sensible prior.  No
%   road user in these scenarios exceeds ~20 m/s, so a 10 m/s standard
%   deviation (100 m^2/s^2) is a wide but finite prior.  Velocity
%   measurements can then be used for UPDATES - where they genuinely help -
%   without an unobserved one poisoning initialisation.
%
%   Process noise is set from the most agile class we model rather than left
%   at the default: a two-wheeler can change velocity quickly, and a filter
%   tuned for cars lags it.  Overestimating agility costs a little smoothness;
%   underestimating it loses the track.
%
%   See also TRACKERWRAPPER, INITCVEKF, DEFAULTCONFIG.

filter = initcvekf(detection);

% --- cap the velocity prior -----------------------------------------------
% initcvekf state layout is [x vx y vy z vz]; velocity sits on 2, 4, 6.
maxVelVar = 100;          % (10 m/s)^2
P = filter.StateCovariance;
for i = [2, 4, 6]
    if P(i, i) > maxVelVar
        P(i, i) = maxVelVar;
    end
end
filter.StateCovariance = P;

% --- process noise --------------------------------------------------------
% Acceleration intensity in m^2/s^3, per axis.  z is held near-still because
% these scenarios are planar.
filter.ProcessNoise = diag([6, 6, 0.1]);
end
