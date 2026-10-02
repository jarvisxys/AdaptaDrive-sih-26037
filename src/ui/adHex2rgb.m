function rgb = adHex2rgb(hex)
%ADHEX2RGB  '#RRGGBB' (or 'RRGGBB') to a 1x3 RGB triplet in [0,1].
%
%   Named with the "ad" prefix so it cannot shadow any current or future
%   MathWorks function of the same purpose.

hex = char(hex);
if ~isempty(hex) && hex(1) == '#'
    hex = hex(2:end);
end
if numel(hex) ~= 6
    error('adHex2rgb:badInput', 'Expected 6 hex digits, got "%s".', hex);
end
rgb = double([hex2dec(hex(1:2)), hex2dec(hex(3:4)), hex2dec(hex(5:6))]) / 255;
end
