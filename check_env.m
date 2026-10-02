function env = check_env(varargin)
%CHECK_ENV  Probe the installed MATLAB environment and select AdaptaDrive backends.
%
%   ENV = CHECK_ENV() returns a struct describing the platform, the installed
%   toolboxes, and the backend that each AdaptaDrive component will use.
%
%   CHECK_ENV with no output argument prints the environment table.
%
%   Name-value options:
%       'force'  (false)  re-probe instead of using the cached result
%       'print'  ([])     force printing on/off; default is (nargout == 0)
%
%   HONESTY CONTRACT
%   A toolbox is reported USABLE only when all three independent probes agree:
%     1. it appears in VER                      -> it is installed
%     2. LICENSE('test', feature) returns true   -> a licence is available
%     3. a representative entry point resolves   -> it can actually be called
%   Each probe is printed as its own column, so a partial install is visible
%   rather than hidden behind a single "yes".
%
%   Two measured quirks of this installation drive that design (see
%   docs/DEVIATIONS.md):
%     * LICENSE('test', ...) ERRORS when the feature name is 28 characters or
%       longer, so every licence probe is wrapped in try/catch.
%     * LICENSE('test','Neural_Network_Toolbox') returns TRUE on this machine
%       even though Deep Learning Toolbox is NOT installed.  Licence alone is
%       therefore never sufficient evidence.
%
%   Nothing here is assumed from documentation or memory; every row is the
%   result of a call made on this machine at run time.
%
%   See also STARTUP, DEFAULTCONFIG.

p = inputParser;
p.addParameter('force', false, @(x) islogical(x) || isnumeric(x));
p.addParameter('print', [],    @(x) isempty(x) || islogical(x) || isnumeric(x));
p.parse(varargin{:});

doPrint = p.Results.print;
if isempty(doPrint)
    doPrint = (nargout == 0);
end
doPrint = logical(doPrint);

persistent cachedEnv
if isempty(cachedEnv) || logical(p.Results.force)
    cachedEnv = probeEnvironment();
end
e = cachedEnv;

if doPrint
    printEnvironment(e);
end

if nargout > 0
    env = e;
end
end

% ------------------------------------------------------------------------
function e = probeEnvironment()
%PROBEENVIRONMENT  Run every probe and assemble the environment struct.

e = struct();
e.timestamp = datetime('now');
e.platform  = platformInfo();

% --- toolbox specification table ------------------------------------------
% {ver name, licence feature (<28 chars), entry point, role in AdaptaDrive, required}
spec = {
  'MATLAB',                             'MATLAB',                     'rng',                 'core',                          true
  'Automated Driving Toolbox',          'Automated_Driving_Toolbox',  'drivingScenario',     'scenario + sensor models',      false
  'Navigation Toolbox',                 'Navigation_Toolbox',         'plannerHybridAStar',  'global planner',                false
  'Sensor Fusion and Tracking Toolbox', 'Sensor_Fusion_and_Tracking', 'trackerGNN',          'tracking (optional extras)',    false
  'Simulink',                           'SIMULINK',                   'new_system',          'behaviour chart host',          false
  'Stateflow',                          'Stateflow',                  'sfnew',               'behaviour chart (display)',     false
  'Parallel Computing Toolbox',         'Distrib_Computing_Toolbox',  'parpool',             'experiment parfor',             false
  'Computer Vision Toolbox',            'Video_and_Image_Blockset',   'pointCloud',          'lidar layer (optional)',        false
  'Image Processing Toolbox',           'Image_Toolbox',              'imdilate',            'risk-map inflation (optional)', false
  'Deep Learning Toolbox',              'Neural_Network_Toolbox',     'trainnet',            'M9 learned prediction',         false
  };

vinfo = ver;
tb = struct('name', {}, 'installed', {}, 'version', {}, 'licensed', {}, ...
            'licenseNote', {}, 'entryPoint', {}, 'entryResolves', {}, ...
            'usable', {}, 'role', {}, 'required', {});

for k = 1:size(spec, 1)
    tb(k) = probeToolbox(vinfo, spec{k, 1}, spec{k, 2}, spec{k, 3}, spec{k, 4}, spec{k, 5});
end
e.toolboxes = tb;

% RoadRunner is a separate desktop application, not a MATLAB toolbox.  All we
% can honestly probe from here is the MATLAB-side import API.
e.roadrunner = struct( ...
    'importApiAvailable', exist('roadrunnerHDMap', 'file') > 0, ...
    'note', 'MATLAB-side import API only; the RoadRunner application itself is not probed');

e.backends = selectBackends(e);
e.warnings = collectWarnings(e);
e.gitCommit = gitCommit();
end

% ------------------------------------------------------------------------
function t = probeToolbox(vinfo, name, feature, entryPoint, role, required)
%PROBETOOLBOX  Three independent probes for one toolbox.

t.name = name;
t.role = role;
t.required = required;

% Probe 1: installed?
idx = find(strcmp({vinfo.Name}, name), 1);
t.installed = ~isempty(idx);
if t.installed
    t.version = vinfo(idx).Version;
else
    t.version = '-';
end

% Probe 2: licensed?  Guarded: license() errors on long feature names.
t.licenseNote = '';
try
    t.licensed = logical(license('test', feature));
catch ME
    t.licensed = false;
    t.licenseNote = ME.message;
end

% Probe 3: does the entry point actually resolve?
t.entryPoint = entryPoint;
t.entryResolves = exist(entryPoint, 'file') > 0 || exist(entryPoint, 'builtin') > 0;

t.usable = t.installed && t.licensed && t.entryResolves;
end

% ------------------------------------------------------------------------
function b = selectBackends(e)
%SELECTBACKENDS  Choose an implementation for each toolbox-dependent component.
%
%   Every component has a pure-MATLAB fallback, so AdaptaDrive runs on a bare
%   MATLAB installation.  The fallback is selected automatically here; the UI
%   Architecture tab reads these fields so the audience can see which backend
%   produced a given run.

has = @(n) isUsable(e, n);

adt  = has('Automated Driving Toolbox');
nav  = has('Navigation Toolbox');
sfx  = has('Stateflow') && has('Simulink');
pct  = has('Parallel Computing Toolbox');

% Scenario container: drivingScenario is a convenience, not a requirement.
if adt
    b.scenario = 'drivingScenario';
else
    b.scenario = 'native';
end

% Sensors: toolbox generators vs our synthetic model (same detection contract).
if adt
    b.sensors = 'toolbox';
else
    b.sensors = 'synthetic';
end

% Tracker: multiObjectTracker ships with ADT.
if adt && exist('multiObjectTracker', 'file') > 0
    b.tracker = 'multiObjectTracker';
else
    b.tracker = 'simpleKF';
end

% Global planner: Hybrid A* -> RRT* -> own grid A*.
if nav && exist('plannerHybridAStar', 'file') > 0
    b.globalPlanner = 'hybridAStar';
elseif nav && exist('plannerRRTStar', 'file') > 0
    b.globalPlanner = 'rrtStar';
else
    b.globalPlanner = 'gridAStar';
end

% Global-planner fallback used on timeout/failure.
if nav && exist('plannerRRTStar', 'file') > 0
    b.globalPlannerFallback = 'rrtStar';
else
    b.globalPlannerFallback = 'gridAStar';
end

% Pure pursuit: toolbox controller vs our own implementation.
if exist('controllerPurePursuit', 'file') > 0
    b.purePursuit = 'toolbox';
else
    b.purePursuit = 'native';
end

% Behaviour: the MATLAB class FSM is ALWAYS the runtime.  The Stateflow chart
% is a generated display/parity artefact only.
b.behaviorRuntime = 'BehaviorFSM';
if sfx
    b.behaviorChart = 'stateflow';
else
    b.behaviorChart = 'unavailable';
end

b.parallel = pct;

% Lidar stays off by default: it costs time and only feeds the static layer.
b.lidar = false;
b.lidarAvailable = adt && exist('lidarPointCloudGenerator', 'file') > 0;

b.videoProfile = pickVideoProfile();
end

% ------------------------------------------------------------------------
function tf = isUsable(e, name)
idx = find(strcmp({e.toolboxes.name}, name), 1);
tf = ~isempty(idx) && e.toolboxes(idx).usable;
end

% ------------------------------------------------------------------------
function prof = pickVideoProfile()
%PICKVIDEOPROFILE  MPEG-4 if this platform offers it, else Motion JPEG AVI.
prof = 'Motion JPEG AVI';
try
    names = {VideoWriter.getProfiles().Name};
    if any(strcmp(names, 'MPEG-4'))
        prof = 'MPEG-4';
    end
catch
    % Leave the AVI default; make_video reports the profile it used.
end
end

% ------------------------------------------------------------------------
function w = collectWarnings(e)
%COLLECTWARNINGS  Honest statements about what this machine cannot do.
w = string.empty(0, 1);

for k = 1:numel(e.toolboxes)
    t = e.toolboxes(k);
    if t.required && ~t.usable
        w(end+1, 1) = sprintf('REQUIRED toolbox not usable: %s', t.name); %#ok<AGROW>
    elseif t.installed && ~t.licensed
        w(end+1, 1) = sprintf('%s is installed but its licence did not test true (%s)', ...
            t.name, t.licenseNote); %#ok<AGROW>
    elseif ~t.installed && t.licensed
        w(end+1, 1) = sprintf(['%s is NOT installed although its licence tests true ' ...
            '- licence alone is not evidence of availability'], t.name); %#ok<AGROW>
    end
end

if strcmp(e.backends.behaviorChart, 'unavailable')
    w(end+1, 1) = "Stateflow/Simulink unavailable: the behaviour chart will not be generated " + ...
        "and testStateflowParity will report SKIPPED (not passed)."; %#ok<AGROW>
end
if ~e.backends.parallel
    w(end+1, 1) = "Parallel Computing Toolbox unavailable: run_experiments will run serially."; %#ok<AGROW>
end
if ~strcmp(e.backends.videoProfile, 'MPEG-4')
    w(end+1, 1) = "MPEG-4 profile unavailable: make_video will fall back to " + ...
        string(e.backends.videoProfile) + "."; %#ok<AGROW>
end
end

% ------------------------------------------------------------------------
function info = platformInfo()
%PLATFORMINFO  Facts recorded into every results file for reproducibility.

info.matlabRelease = version('-release');
info.matlabVersion = version;
info.computer      = computer;
info.numCores      = feature('numcores');   % PHYSICAL cores, not logical
info.hostname      = getHostname();
info.cpu           = getCpuName();
info.ramGB         = getRamGB();
end

% ------------------------------------------------------------------------
function gb = getRamGB()
%GETRAMGB  Installed physical memory, or NaN when it cannot be measured.
%   Reported rather than assumed: the build spec named a 32 GB target machine,
%   and this one has considerably less (see docs/DEVIATIONS.md D8).
gb = NaN;
try
    [~, sv] = memory;                       %#ok<MEMOR> Windows only
    gb = sv.PhysicalMemory.Total / 2^30;
catch
    % Non-Windows, or memory() unavailable: leave NaN and print "unknown".
end
end

% ------------------------------------------------------------------------
function name = getCpuName()
%GETCPUNAME  Real CPU model string.  Timing claims must name the hardware they
%   were measured on, so this is read from the machine, never assumed.
name = '';
try
    name = strtrim(winqueryreg('HKEY_LOCAL_MACHINE', ...
        'HARDWARE\DESCRIPTION\System\CentralProcessor\0', 'ProcessorNameString'));
catch
    % Not Windows, or the key is unavailable.
end
if isempty(name)
    name = strtrim(getenv('PROCESSOR_IDENTIFIER'));
end
if isempty(name)
    name = 'unknown CPU';
end
end

% ------------------------------------------------------------------------
function h = getHostname()
h = strtrim(getenv('COMPUTERNAME'));
if isempty(h)
    h = strtrim(getenv('HOSTNAME'));
end
if isempty(h)
    h = 'unknown host';
end
end

% ------------------------------------------------------------------------
function c = gitCommit()
%GITCOMMIT  Short commit hash, or 'not a git repository'.
c = 'not a git repository';
try
    [status, out] = system('git rev-parse --short HEAD');
    if status == 0
        out = strtrim(out);
        if ~isempty(out)
            c = out;
        end
    end
catch
    % Leave the default.
end
end

% ------------------------------------------------------------------------
function printEnvironment(e)
%PRINTENVIRONMENT  The M0 table.

line = repmat('-', 1, 96);

fprintf('\n');
fprintf('%s\n', line);
fprintf('  AdaptaDrive - environment check   (team SECOND INNINGS, SIH 2026, PS 26037)\n');
fprintf('%s\n', line);
fprintf('  MATLAB   : R%s  (%s)  on %s\n', e.platform.matlabRelease, ...
    strtok(e.platform.matlabVersion), e.platform.computer);
if isnan(e.platform.ramGB)
    ramStr = 'RAM unknown';
else
    ramStr = sprintf('%.1f GB RAM', e.platform.ramGB);
end
fprintf('  CPU      : %s  (%d physical cores, %s)\n', ...
    e.platform.cpu, e.platform.numCores, ramStr);
fprintf('  Host     : %s\n', e.platform.hostname);
fprintf('  Git      : %s\n', e.gitCommit);
fprintf('  Probed   : %s\n', string(e.timestamp, 'yyyy-MM-dd HH:mm:ss'));
fprintf('%s\n', line);

fprintf('  %-36s %-9s %-9s %-21s %-6s\n', ...
    'TOOLBOX', 'INSTALLED', 'LICENSED', 'ENTRY POINT', 'USABLE');
fprintf('  %-36s %-9s %-9s %-21s %-6s\n', ...
    repmat('-', 1, 36), repmat('-', 1, 9), repmat('-', 1, 9), ...
    repmat('-', 1, 21), repmat('-', 1, 6));

for k = 1:numel(e.toolboxes)
    t = e.toolboxes(k);
    if t.installed
        instCol = t.version;
    else
        instCol = 'no';
    end
    if t.entryResolves
        entryCol = t.entryPoint;
    else
        entryCol = sprintf('%s (missing)', t.entryPoint);
    end
    fprintf('  %-36s %-9s %-9s %-21s %-6s\n', ...
        t.name, instCol, yesno(t.licensed), entryCol, yesno(t.usable));
end

fprintf('%s\n', line);
fprintf('  SELECTED BACKENDS (automatic; every component has a pure-MATLAB fallback)\n');
fprintf('%s\n', line);
b = e.backends;
printBackend('Scenario container', b.scenario,             'drivingScenario');
printBackend('Sensors',            b.sensors,              'toolbox');
printBackend('Tracker',            b.tracker,              'multiObjectTracker');
printBackend('Global planner',     b.globalPlanner,        'hybridAStar');
printBackend('  fallback planner', b.globalPlannerFallback, 'rrtStar');
printBackend('Pure pursuit',       b.purePursuit,          'toolbox');
printBackend('Behaviour runtime',  b.behaviorRuntime,      'BehaviorFSM');
printBackend('Behaviour chart',    b.behaviorChart,        'stateflow');
printBackend('Video profile',      b.videoProfile,         'MPEG-4');
fprintf('  %-22s : %s\n', 'Parallel (parfor)', ternary(b.parallel, 'available', 'serial fallback'));
fprintf('  %-22s : %s\n', 'Lidar', ternary(b.lidar, 'on', ...
    ternary(b.lidarAvailable, 'off by default (available)', 'off by default (unavailable)')));
fprintf('  %-22s : %s\n', 'RoadRunner import API', ternary(e.roadrunner.importApiAvailable, ...
    'present (application itself not probed)', 'absent'));

if ~isempty(e.warnings)
    fprintf('%s\n', line);
    fprintf('  NOTES\n');
    for k = 1:numel(e.warnings)
        fprintf('   !  %s\n', e.warnings(k));
    end
end

fprintf('%s\n', line);
fprintf('  All figures produced by this project are SIMULATION results.\n');
fprintf('  Architectural coverage is not validated performance (research doc, 35.1).\n');
fprintf('%s\n\n', line);
end

% ------------------------------------------------------------------------
function printBackend(label, chosen, preferred)
if strcmp(chosen, preferred)
    tag = '';
else
    tag = sprintf('   [fallback; preferred "%s" unavailable]', preferred);
end
fprintf('  %-22s : %s%s\n', label, chosen, tag);
end

% ------------------------------------------------------------------------
function s = yesno(tf)
if tf
    s = 'yes';
else
    s = 'NO';
end
end

% ------------------------------------------------------------------------
function s = ternary(tf, a, b)
if tf
    s = a;
else
    s = b;
end
end
