function T = run_stress(varargin)
%RUN_STRESS  Requirement B5 stress conditions, measured and reported apart.
%
%   Three conditions, each a single change from the nominal PROPOSED run so
%   that any difference is attributable:
%
%     density x2      twice the agents (RUN_EXPERIMENTS 'density')
%     sensor stress   20% camera dropout and 0.8 m position sigma
%                     (configPreset 'STRESS-sensor'); radars untouched, so a
%                     drop in performance says something about which sensor the
%                     stack depends on
%     unseen variant  the market plus a wrong-way bus and a crossing cow
%                     (BUILDSCENARIO 'unseen'); nothing was tuned on it
%
%   Results go to results/stress/ and are reported SEPARATELY. They are never
%   pooled into the headline completion rate: a stress row averaged into a
%   nominal table would understate nominal performance and overstate
%   robustness at the same time.
%
%   RUN_STRESS('scenarios', {...}) restricts the scenario set. The default is
%   chosen from the measured matrix, not by guesswork: PROPOSED completes
%   intersection and highway at 100% and cattle at 50%, so a degradation there
%   has room to show. Market is excluded because every configuration scores 0%
%   on it - stressing a scenario that already fails completely can only produce
%   0% again, which costs machine time to learn nothing. Village is excluded as
%   the slowest scenario by a wide margin (its runs average ~165 s against ~40 s
%   for cattle); it is the omission most worth adding back given more time.
%
%   See also RUN_EXPERIMENTS, CONFIGPRESET, SCENARIOUNSEEN.

p = inputParser;
p.addParameter('scenarios', {'intersection', 'highway', 'cattle'});
p.addParameter('seeds', 1:2);
p.addParameter('outDir', adRoot('results', 'stress'));
p.parse(varargin{:});
opt = p.Results;

if ~isfolder(opt.outDir), mkdir(opt.outDir); end

line = repmat('=', 1, 78);
fprintf('\n%s\n  AdaptaDrive B5 stress conditions  (SIMULATION)\n%s\n', line, line);

blocks = {};

% ---------------------------------------------------------------- nominal
% The comparison baseline is re-run here rather than read from the main
% matrix, so nominal and stressed rows come from the same machine state.
% Latency especially is not comparable across sessions on a 7.7 GB machine.
fprintf('\n  --- nominal (baseline for comparison) ---\n');
blocks{end+1} = tag(run_experiments('scenarios', opt.scenarios, ...
    'configs', {'PROPOSED'}, 'seeds', opt.seeds, 'density', 1.0, ...
    'outDir', fullfile(opt.outDir, 'nominal')), 'nominal');

% ---------------------------------------------------------------- density
fprintf('\n  --- density x2 ---\n');
blocks{end+1} = tag(run_experiments('scenarios', opt.scenarios, ...
    'configs', {'PROPOSED'}, 'seeds', opt.seeds, 'density', 2.0, ...
    'outDir', fullfile(opt.outDir, 'density2')), 'density x2');

% ---------------------------------------------------------------- sensor
fprintf('\n  --- sensor stress ---\n');
blocks{end+1} = tag(run_experiments('scenarios', opt.scenarios, ...
    'configs', {'STRESS-sensor'}, 'seeds', opt.seeds, 'density', 1.0, ...
    'outDir', fullfile(opt.outDir, 'sensor')), 'sensor stress');

% ---------------------------------------------------------------- unseen
% Two configurations, because the only useful question about a held-out case is
% whether the proposed system does better on it than the simplest baseline.
fprintf('\n  --- unseen variant (market + wrong-way bus + crossing cow) ---\n');
blocks{end+1} = tag(run_experiments('scenarios', {'unseen'}, ...
    'configs', {'PROPOSED', 'BL1'}, 'seeds', opt.seeds, 'density', 1.0, ...
    'outDir', fullfile(opt.outDir, 'unseen')), 'unseen');

T = vertcat(blocks{:});
writetable(T, fullfile(opt.outDir, 'stress_summary.csv'));

fprintf('\n%s\n  B5 STRESS SUMMARY  (each condition measured, none pooled)\n%s\n', ...
    line, line);
% Config is printed as well as scenario. Without it the two unseen-variant rows
% (PROPOSED and BL1) are indistinguishable, which is exactly the comparison the
% held-out case exists to make.
fprintf('  %-14s %-13s %-13s %8s %5s %8s %7s\n', ...
    'condition', 'scenario', 'config', 'complete', 'n', 'minClear', 'p95 ms');
fprintf('  %s\n', repmat('-', 1, 78));
for k = 1:height(T)
    fprintf('  %-14s %-13s %-13s %7.0f%% %5d %8.2f %7.0f\n', ...
        T.condition(k), T.scenario(k), T.config(k), ...
        100*T.completionRate(k), T.n(k), ...
        T.minClearance_mean(k), T.latencyP95_mean(k));
end
fprintf('%s\n', line);
fprintf(['\n  These rows are NOT part of the headline completion rate.\n' ...
         '  Stress results belong beside the nominal ones, not inside them.\n\n']);
end

% ========================================================================
function S = tag(S, name)
%TAG  Label a summary block with the condition that produced it.
%   Without this the merged table would have two identical-looking PROPOSED
%   rows per scenario and no way to tell which was stressed.
S.condition = repmat(string(name), height(S), 1);
S = movevars(S, 'condition', 'Before', 1);
end
