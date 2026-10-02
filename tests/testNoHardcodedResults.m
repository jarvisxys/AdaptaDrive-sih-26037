classdef testNoHardcodedResults < matlab.unittest.TestCase
    %TESTNOHARDCODEDRESULTS  The overriding rule, enforced by a test.
    %
    %   Every number shown anywhere must be computed by running the
    %   simulation.  This test is the automated half of that rule:
    %
    %     1. a static scan of the UI for numeric literals assigned to metric
    %        display fields (thresholds and layout constants are allowed);
    %     2. the Results tab must render honestly with NO results file, saying
    %        so rather than showing anything;
    %     3. MetricsLogger must initialise unmeasured metrics to NaN, never 0.
    %
    %   The scan is deliberately simple and will not catch every possible
    %   fabrication.  It catches the easy and most likely one - a plausible
    %   number typed into a label to make a screenshot look finished.

    methods (Test)

        function uiHasNoHardcodedMetricText(tc)
            % Look for metric-looking literals assigned into label text.
            files = [dir(fullfile(adRoot('src', 'ui'), '*.m')); ...
                     dir(fullfile(adRoot(), 'AdaptaDriveApp.m'))];
            tc.assertNotEmpty(files, 'No UI source found to scan.');

            % The failure mode this guards against is precise: a literal
            % number with a unit written into something a viewer READS as a
            % measurement - a label's Text, or a 'String' passed to a
            % graphics object.  Scanning for numbers generally is useless
            % here: a UI is made of numbers (positions, widths, colours,
            % font sizes), and an earlier broad pattern flagged 1018 lines,
            % which is the same as flagging nothing.
            patterns = { ...
                '\.Text\s*=\s*''[^'']*\d[^'']*''', ...
                '''String''\s*,\s*''[^'']*\d+\.?\d*\s*(m/s|ms|m\b|s\b|%)[^'']*'''};

            offenders = {};
            for k = 1:numel(files)
                f = fullfile(files(k).folder, files(k).name);
                lines = splitlines(string(fileread(f)));
                for i = 1:numel(lines)
                    L = char(strtrim(lines(i)));
                    if isempty(L) || L(1) == '%'
                        continue        % comments may discuss numbers freely
                    end
                    % sprintf/format strings are how REAL values are shown.
                    if contains(L, 'sprintf') || contains(L, 'fprintf')
                        continue
                    end
                    % Thresholds and axis furniture are explicitly allowed:
                    % the rule is "no fabricated MEASUREMENTS", not "no
                    % numbers".  A dashed line labelled "200 ms" is the target
                    % being drawn, not a result being claimed.
                    if contains(L, 'yline') || contains(L, 'xline') || ...
                            contains(L, 'Limits') || contains(L, 'lim(')
                        continue
                    end
                    for pk = 1:numel(patterns)
                        if ~isempty(regexp(L, patterns{pk}, 'once'))
                            offenders{end+1} = sprintf('%s:%d  %s', ...
                                files(k).name, i, L); %#ok<AGROW>
                            break
                        end
                    end
                end
            end

            tc.verifyEmpty(offenders, sprintf( ...
                ['UI contains literal metric-looking text. Every displayed ' ...
                 'number must come from a run:\n  %s'], ...
                strjoin(offenders, sprintf('\n  '))));
        end

        function resultsTabIsHonestWithoutData(tc)
            % With no runs.csv the Results tab must say there are no results.
            % It must not render an empty chart that could be mistaken for a
            % measurement of zero.
            src = fileread(fullfile(adRoot(), 'AdaptaDriveApp.m'));

            tc.verifyTrue(contains(src, 'No experiment results yet'), ...
                'Results tab has no explicit "no results" state.');
            tc.verifyTrue(contains(src, 'isfile(runsFile)'), ...
                'Results tab does not check whether a results file exists.');
            tc.verifyTrue(contains(src, 'never displays placeholder numbers'), ...
                'Results tab does not state that it shows only measured data.');
        end

        function unmeasuredMetricsAreNaNNotZero(tc)
            % A zero reads as a good score. NaN prints as "not measured" and
            % does not average into a summary.
            m = MetricsLogger.blank();
            numericFields = {'completionTime', 'pathLength', 'minClearance', ...
                'minTTC', 'hazardTraversals', 'minPotholeClearance', ...
                'jerkRMS', 'jerkMax', 'latAccRMS', 'curvatureMean', ...
                'emergencyBrakes', 'wrongWayFlags', 'mergeDetections', ...
                'replanCount', 'latencyP50', 'latencyP95', 'latencyMax'};
            for k = 1:numel(numericFields)
                v = m.(numericFields{k});
                tc.verifyTrue(isnan(v), sprintf( ...
                    'Metric "%s" defaults to %g; it must default to NaN.', ...
                    numericFields{k}, v));
            end
        end

        function metricsComeFromTheLogOnly(tc)
            % MetricsLogger.compute must take the log as its source. If it
            % ever read live simulator state, a metric could be "helped" by
            % the component it is judging.
            src = fileread(fullfile(adRoot('src', 'metrics'), 'MetricsLogger.m'));
            tc.verifyTrue(contains(src, 'PURE FUNCTION OF THE LOG'), ...
                'MetricsLogger no longer documents that it reads only the log.');
            tc.verifyFalse(contains(src, 'scenario.agents('), ...
                'MetricsLogger reads live agent state instead of the log.');
        end

        function everyFigureCarriesProvenance(tc)
            % A latency number with no hardware attached is not a result.
            src = fileread(fullfile(adRoot(), 'make_figures.m'));
            tc.verifyTrue(contains(src, 'SIMULATION'), ...
                'Figures do not label themselves as simulation.');
            tc.verifyTrue(contains(src, 'matlabRelease'), ...
                'Figures do not record the MATLAB release.');
            tc.verifyTrue(contains(src, 'shortCpu'), ...
                'Figures do not record the CPU they were measured on.');
        end

        function experimentRunnerRecordsFailures(tc)
            % A run that errors must appear in the CSV, not vanish from it.
            src = fileread(fullfile(adRoot(), 'run_experiments.m'));
            tc.verifyTrue(contains(src, '"error"') || contains(src, '''error'''), ...
                'run_experiments does not record errored runs.');
            tc.verifyTrue(contains(src, 'never dropped') || ...
                          contains(src, 'Recorded, never dropped'), ...
                'run_experiments does not state that failures are kept.');
            tc.verifyTrue(contains(src, 'wilson'), ...
                'run_experiments does not report a confidence interval.');
        end
    end
end
