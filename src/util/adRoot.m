function p = adRoot(varargin)
%ADROOT  Absolute path of the AdaptaDrive repository root.
%
%   P = ADROOT() returns the repository root directory.
%   P = ADROOT('results','figures') returns a path built under the root.
%
%   Derived from this file's own location (src/util/adRoot.m), so it is
%   independent of the current working directory.

here = fileparts(mfilename('fullpath'));          % .../src/util
p = fileparts(fileparts(here));                   % .../
if nargin > 0
    p = fullfile(p, varargin{:});
end
end
