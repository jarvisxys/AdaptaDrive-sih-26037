function [log, scenario, cfg] = adLoadRun(scenarioName, configName, seed)
%ADLOADRUN  Fetch a saved run, or produce one if it is not on disk.
%
%   [LOG, SCENARIO, CFG] = ADLOADRUN('village', 'PROPOSED', 1)
%
%   Looks for logs/<scenario>_<config>_seed<N>.mat and loads it. If there is
%   no such log, the simulation is run once at FULL log detail and that run is
%   returned instead - the renderers need the risk field and the candidate fan
%   on every cycle, which 'light' logging stores only every fifth cycle.
%
%   The scenario is rebuilt from the log's own metadata rather than from the
%   caller's arguments, so a log always renders against the world it was
%   actually recorded in.
%
%   Shared by MAKE_VIDEO and MAKE_GRID_VIDEO so the two cannot drift apart on
%   how a run is obtained.
%
%   See also MAKE_VIDEO, MAKE_GRID_VIDEO.

if nargin < 3 || isempty(seed), seed = 1; end

f = adRoot('logs', sprintf('%s_%s_seed%d.mat', scenarioName, configName, seed));

if isfile(f)
    S = load(f, 'log');
    log = S.log;
    cfg = log.meta.cfg;
    scenario = buildScenario(log.meta.scenario, log.meta.seed, ...
        log.meta.density, cfg);
else
    fprintf('  no saved log for %s/%s/seed %d - running it now\n', ...
        scenarioName, configName, seed);
    cfg = configPreset(configName, 'scenario', scenarioName, 'seed', seed);
    cfg.io.logDetail = 'full';
    scenario = buildScenario(scenarioName, seed, 1.0, cfg);
    res = SimEngine(scenario, cfg).run();
    log = res.log;
end
end
