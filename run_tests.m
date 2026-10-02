function results = run_tests(varargin)
%RUN_TESTS  Run the AdaptaDrive unit-test suite.
%
%   RUN_TESTS()            runs every test in tests/
%   RUN_TESTS('testFSM')   runs one test file (name or partial name)
%
%   Reporting rule: PASSED, FAILED and SKIPPED are three different outcomes
%   and are printed as three different numbers.  A test that could not run
%   (for example the Stateflow parity test on a machine without Stateflow) is
%   reported SKIPPED and never counted as a pass.
%
%   In "matlab -batch" the function raises an error when anything failed, so
%   the process exit code is non-zero and CI notices.

import matlab.unittest.TestSuite
import matlab.unittest.TestRunner
import matlab.unittest.Verbosity

testsDir = adRoot('tests');
if ~isfolder(testsDir)
    error('run_tests:noTests', 'Test folder not found: %s', testsDir);
end

suite = TestSuite.fromFolder(testsDir, 'IncludingSubfolders', true);

if ~isempty(varargin)
    pattern = char(varargin{1});
    keep = contains({suite.Name}, pattern, 'IgnoreCase', true);
    if ~any(keep)
        error('run_tests:noMatch', 'No test matched "%s".', pattern);
    end
    suite = suite(keep);
end

runner = TestRunner.withTextOutput('OutputDetail', Verbosity.Concise);
res = runner.run(suite);

printSummary(res);

if nargout > 0
    results = res;
end

nFailed = sum([res.Failed]);
if nFailed > 0
    error('run_tests:failures', '%d of %d test(s) FAILED.', nFailed, numel(res));
end
end

% ------------------------------------------------------------------------
function printSummary(res)
line = repmat('-', 1, 96);
passed     = [res.Passed];
failed     = [res.Failed];
incomplete = [res.Incomplete];

fprintf('\n%s\n', line);
fprintf('  AdaptaDrive test summary\n');
fprintf('%s\n', line);
fprintf('  total   : %d\n', numel(res));
fprintf('  passed  : %d\n', sum(passed));
fprintf('  FAILED  : %d\n', sum(failed));
fprintf('  SKIPPED : %d   (not counted as passes)\n', sum(incomplete));
fprintf('  duration: %.2f s\n', sum([res.Duration]));

if any(failed)
    fprintf('%s\n  FAILED tests:\n', line);
    names = {res(failed).Name};
    for k = 1:numel(names)
        fprintf('   x  %s\n', names{k});
    end
end
if any(incomplete)
    fprintf('%s\n  SKIPPED tests:\n', line);
    names = {res(incomplete).Name};
    for k = 1:numel(names)
        fprintf('   -  %s\n', names{k});
    end
end
fprintf('%s\n\n', line);
end
