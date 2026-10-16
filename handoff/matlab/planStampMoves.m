function out = planStampMoves(selectionFile, approachHeight_mm, stepSize_mm, includeContact)
% Turn the vision targets into a checked list of xyz waypoints for the firmware.
% PREVIEW ONLY: this never opens USB and never moves anything. It prints the
% commands you would type, and refuses to print them if the model rejects a point.
%
%   out = planStampMoves('results\2026...\targetSelection.mat')
%   out = planStampMoves(file, 70, 8)   % approach height, max mm per step
%
% The firmware rejects any xyz move more than 10 mm from its current MODEL pose,
% so the path is emitted as many small absolute xyz commands, not two big ones.

if nargin < 2 || isempty(approachHeight_mm), approachHeight_mm = 60; end
if nargin < 3 || isempty(stepSize_mm),       stepSize_mm = 8; end
if nargin < 4 || isempty(includeContact),    includeContact = false; end
assert(stepSize_mm <= 10, 'Firmware rejects steps over 10 mm.');

loaded = load(selectionFile);
cfg = loaded.cfg;
inkPoint_mm   = loaded.targetXYZ_mm(1,:);
paperPoint_mm = loaded.targetXYZ_mm(2,:);

% Where "ready" leaves the arm, per the model (1950/1950/1950).
readyAngles_deg = zeros(1,3);
for motor = 1:3
    readyAngles_deg(motor) = interp1(cfg.ik.pulseFit_us(motor,:), cfg.ik.angleFit_deg(motor,:), 1950);
end
startPoint_mm = rasticraneFk(readyAngles_deg, cfg.ik);

% Waypoints: up over the ink, down to it, back up, over to the paper, down, up.
aboveInk_mm   = [inkPoint_mm(1:2)   approachHeight_mm];
abovePaper_mm = [paperPoint_mm(1:2) approachHeight_mm];
if includeContact
    corners = [startPoint_mm; aboveInk_mm; inkPoint_mm; aboveInk_mm; ...
               abovePaper_mm; paperPoint_mm; abovePaper_mm];
    legNames = {'ready -> above ink','lower to ink','raise from ink', ...
                'above ink -> above paper','lower to paper','raise from paper'};
else
    % Approach heights only: nothing descends to a surface.
    corners = [startPoint_mm; aboveInk_mm; abovePaper_mm; startPoint_mm];
    legNames = {'ready -> above ink','above ink -> above paper','above paper -> ready'};
end

% Break every leg into steps no longer than stepSize_mm.
waypoints_mm = startPoint_mm;
legOfWaypoint = {'start'};
for leg = 1:size(corners,1)-1
    fromPoint = corners(leg,:); toPoint = corners(leg+1,:);
    stepCount = max(1, ceil(norm(toPoint-fromPoint)/stepSize_mm));
    for s = 1:stepCount
        waypoints_mm(end+1,:) = fromPoint + (toPoint-fromPoint)*(s/stepCount); %#ok<AGROW>
        legOfWaypoint{end+1} = legNames{leg}; %#ok<AGROW>
    end
end

% Check every waypoint against the same model the firmware uses.
count = size(waypoints_mm,1);
reachable = false(count,1); inRange = false(count,1);
pulses_us = nan(count,3); reason = strings(count,1);
for k = 1:count
    preview = rasticraneIkPreview(waypoints_mm(k,:), cfg.ik);
    reachable(k) = preview.reachable;
    inRange(k)   = preview.endpointInTestRange;
    pulses_us(k,:) = preview.pulses_us;
    reason(k) = string(preview.reason);
end

out.table = table(string(legOfWaypoint(:)), waypoints_mm(:,1), waypoints_mm(:,2), waypoints_mm(:,3), ...
    reachable, inRange, pulses_us(:,1), pulses_us(:,2), pulses_us(:,3), reason, ...
    'VariableNames', {'leg','X_mm','Y_mm','Z_mm','reachable','inTestRange','M1_us','M2_us','base_us','reason'});
disp(out.table);

out.waypoints_mm = waypoints_mm;
out.allApproved = all(reachable & inRange);

if out.allApproved
    fprintf('\n--- type these one at a time, waiting for DONE after each ---\n');
    fprintf('ready\n');
    for k = 2:count
        fprintf('xyz %.1f %.1f %.1f\n', waypoints_mm(k,1), waypoints_mm(k,2), waypoints_mm(k,3));
    end
    fprintf(['\nEvery point passed the model checks. That is geometry only: no collision, ' ...
             'no stamp in the magnet, no proof the arm physically arrives.\n']);
else
    fprintf('\nNO COMMANDS EMITTED. Blocking points:\n');
    blocked = find(~(reachable & inRange));
    for k = blocked(:).'
        fprintf('  %-26s [%7.1f %7.1f %6.1f]  %s\n', legOfWaypoint{k}, ...
            waypoints_mm(k,1), waypoints_mm(k,2), waypoints_mm(k,3), reason(k));
    end
    fprintf(['\nFix the model/firmware limits that caused these, re-run, and only then move.\n']);
end
end
