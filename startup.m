function env = startup()
%STARTUP  Put AdaptaDrive on the MATLAB path and probe the environment.
%
%   Run from the repository root:
%       matlab -batch "startup; check_env"
%
%   Adds the repository root, src/ (recursively) and tests/ to the path,
%   creates the generated-output folders, and probes the installed toolboxes
%   via CHECK_ENV.  The probe result is cached inside CHECK_ENV, so calling
%   CHECK_ENV again afterwards is free and prints the table.
%
%   ENV = STARTUP() also returns the environment struct.
%
%   See also CHECK_ENV, DEFAULTCONFIG, RUN_TESTS.

root = fileparts(mfilename('fullpath'));

addpath(root);
addpath(genpath(fullfile(root, 'src')));
addpath(fullfile(root, 'tests'));

% Generated outputs live here.  They are produced by runs, never checked in.
for d = ["results", "logs"]
    p = fullfile(root, d);
    if ~isfolder(p)
        mkdir(p);
    end
end

% Probe the environment once and cache it.  Printing is left to the caller so
% that "startup" stays quiet inside scripts.
e = check_env('print', false);

if nargout > 0
    env = e;
end
end
