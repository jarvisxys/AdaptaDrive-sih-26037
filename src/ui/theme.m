function t = theme()
%THEME  The single source of colour and typography for AdaptaDrive.
%
%   Every figure, panel, legend and chart reads from here, so a class has the
%   same colour in the bird's-eye view, the legend and the results charts.
%
%   Palette follows the team deck: navy panels, off-white text, SIH saffron
%   accent, and the safe/caution/danger triad.
%
%   See also ADCLASSCOLOR, ADHEX2RGB, BEVRENDERER.

persistent cached
if ~isempty(cached)
    t = cached;
    return
end

h = @adHex2rgb;

% --- surfaces -------------------------------------------------------------
t.bg          = h('#12203C');   % window background, one step darker than panels
t.panel       = h('#1B2A4A');   % deck navy: cards and panels
t.panelAlt    = h('#22335A');   % raised card / table header
t.border      = h('#2E4270');
t.grid        = h('#2A3A5E');   % axes gridlines

% --- text -----------------------------------------------------------------
t.text        = h('#F5F7FA');   % off-white
t.textDim     = h('#9AA5B1');
t.textFaint   = h('#6B7785');

% --- semantic -------------------------------------------------------------
t.accent      = h('#E8792B');   % SIH saffron: ego, global path, primary action
t.safe        = h('#2E9E5B');   % chosen trajectory, success
t.danger      = h('#D64545');   % collision, wrong-way, emergency
t.caution     = h('#F2B134');   % warnings, potholes

% --- world rendering ------------------------------------------------------
t.drivable    = h('#2B2F3A');   % dark grey drivable surface
t.offroad     = h('#151A24');
t.roadEdge    = h('#4A5568');
t.pothole     = h('#E8792B');
t.stall       = h('#1B2A4A');
t.ego         = h('#E8792B');
t.globalPath  = h('#E8792B');
t.localTraj   = h('#2E9E5B');
t.candidate   = h('#7FA8D9');   % DWA candidate fan (drawn at low alpha)

% --- risk-map colormap: transparent -> amber -> red -----------------------
% Alpha carries the magnitude (AlphaData = R), so the colours only need to
% shift hue.  Built once here so the BEV and the risk figures cannot diverge.
n = 256;
u = linspace(0, 1, n)';
lo = h('#F2B134');    % caution amber at low risk
hi = h('#D64545');    % danger red at high risk
t.riskColormap = (1 - u) .* lo + u .* hi;
t.riskAlphaMax = 0.78;          % never fully hide the world underneath

% --- class colours (deck legend) -----------------------------------------
% Row k is class k; row 8 is the unknown class (classId 0).
t.classFill = [
    h('#4A90D9')      % 1 car
    h('#6C5CE7')      % 2 bus
    h('#F2B134')      % 3 auto-rickshaw
    h('#00B8A9')      % 4 two-wheeler
    h('#E84393')      % 5 pedestrian
    h('#A0785A')      % 6 pushcart
    h('#FFFFFF')      % 7 cow  (needs the dark outline below)
    h('#9AA5B1')      % - unknown
    ];
t.classEdge = repmat(h('#0E1626'), 8, 1);   % dark outline for every class
t.classNames = {'car', 'bus', 'auto', 'twowheeler', 'pedestrian', ...
                'pushcart', 'cow', 'unknown'};

% --- FSM state colours (deck order) --------------------------------------
t.stateOrder = {'CRUISE', 'SLOW_DOWN', 'YIELD', 'OVERTAKE_MERGE', ...
                'REJOIN', 'STOP', 'EMERGENCY_BRAKE'};
t.stateColor = [
    h('#2E9E5B')      % CRUISE          green
    h('#F2B134')      % SLOW_DOWN       amber
    h('#E8792B')      % YIELD           saffron
    h('#4A90D9')      % OVERTAKE_MERGE  blue
    h('#00B8A9')      % REJOIN          teal
    h('#9AA5B1')      % STOP            grey
    h('#D64545')      % EMERGENCY_BRAKE red
    ];

% --- typography -----------------------------------------------------------
t.font       = pickFont({'Segoe UI', 'Helvetica', 'Arial'});
t.fontMono   = pickFont({'Consolas', 'Courier New'});
t.fsTitle    = 20;
t.fsHeading  = 14;
t.fsBody     = 12;
t.fsSmall    = 10;
t.fsMetric   = 26;

cached = t;
end

% ------------------------------------------------------------------------
function name = pickFont(candidates)
%PICKFONT  First candidate this machine actually has, else MATLAB's default.
persistent available
if isempty(available)
    try
        available = listfonts;
    catch
        available = {};
    end
end
name = candidates{end};
for k = 1:numel(candidates)
    if any(strcmpi(available, candidates{k}))
        name = candidates{k};
        return
    end
end
end
