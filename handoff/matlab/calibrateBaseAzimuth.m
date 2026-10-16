function out = calibrateBaseAzimuth(photoFolder, sweepTagId, sweepTagHeight_mm)
% Fit base servo PULSE -> AZIMUTH from a set of photos taken during a base sweep.
%
%   out = calibrateBaseAzimuth('C:\...\baseSweep', 4, 18.5)
%
% How to take the photos:
%   1. Tape a unique, FLAT, horizontal AprilTag on the rotating platform/arm,
%      far enough from the axis to swing a good arc, close enough to stay in frame.
%      Measure its height above the table in mm -> sweepTagHeight_mm.
%   2. Do NOT move the camera, the base reference tag, or anything else between shots.
%   3. For each base pulse, send "3 <us>" over serial, let it settle, take one photo,
%      and name the file with that pulse in it, e.g. base_1300.jpg, base_1450.jpg ...
%      Include a shot at 1900 (home) -- it defines zero azimuth.
%
% The tag does not have to sit on the arm's centerline. Its azimuth is measured
% relative to the azimuth at pulse 1900, so any constant mounting offset drops out.

if nargin < 3
    error('Usage: calibrateBaseAzimuth(photoFolder, sweepTagId, sweepTagHeight_mm)');
end

files = [dir(fullfile(photoFolder,'*.jpg')); dir(fullfile(photoFolder,'*.jpeg')); ...
         dir(fullfile(photoFolder,'*.png'))];
assert(~isempty(files), 'No photos found in %s', photoFolder);

% Config with the sweep tag added as an extra object.
cfg = visionConfig();
cfg.maxPoseFit_px = 8;   % independent tag geometry check below is the real gate
newObject = struct('name','baseSweepTag','id',sweepTagId, ...
                   'tagHeight_mm',sweepTagHeight_mm,'targetOffsetTag_mm',[NaN NaN NaN]);
cfg.objects(end+1) = newObject;

pulse_us    = nan(numel(files),1);
azimuth_deg = nan(numel(files),1);
sizeRms_mm  = nan(numel(files),1);
fileName    = strings(numel(files),1);

for k = 1:numel(files)
    fileName(k) = string(files(k).name);
    digits = regexp(files(k).name, '(\d{3,4})', 'tokens', 'once');
    if isempty(digits)
        fprintf('SKIP %s: no pulse number in the filename.\n', files(k).name); continue
    end
    pulse_us(k) = str2double(digits{1});

    result = analyzeSetup(fullfile(photoFolder, files(k).name), cfg);
    if ~strcmp(result.status,'CALIBRATED_ANALYSIS_PREVIEW_ONLY') || isempty(result.objects)
        fprintf('SKIP %s: run status %s\n', files(k).name, result.status); continue
    end
    index = find([result.objects.id] == sweepTagId);
    if isempty(index)
        fprintf('SKIP %s: sweep tag %d not detected.\n', files(k).name, sweepTagId); continue
    end
    tag = result.objects(index);
    if ~any(strcmp(tag.status,{'TAG_LOCATED_TARGET_OFFSET_REQUIRED','TARGET_LOCATED_PREVIEW_ONLY'}))
        fprintf('SKIP %s: sweep tag status %s\n', files(k).name, tag.status); continue
    end
    xy = tag.tagCenterRobot_mm(1:2);
    azimuth_deg(k) = atan2d(xy(2), xy(1));
    sizeRms_mm(k)  = tag.sizeRms_mm;
    fprintf('%-24s pulse %4d us -> raw azimuth %7.2f deg (tag size rms %.2f mm)\n', ...
        files(k).name, pulse_us(k), azimuth_deg(k), sizeRms_mm(k));
end

good = isfinite(pulse_us) & isfinite(azimuth_deg);
assert(nnz(good) >= 3, 'Need at least 3 usable photos; got %d.', nnz(good));
pulse_us = pulse_us(good); azimuth_deg = azimuth_deg(good);
sizeRms_mm = sizeRms_mm(good); fileName = fileName(good);

% Sort by pulse and unwrap, so a sweep crossing +/-180 deg stays continuous.
[pulse_us, order] = sort(pulse_us);
azimuth_deg = azimuth_deg(order);
sizeRms_mm  = sizeRms_mm(order);
fileName    = fileName(order);
azimuth_deg = rad2deg(unwrap(deg2rad(azimuth_deg)));

% Straight-line fit, then shift so pulse 1900 is exactly 0 deg (the robot +X axis).
coefficients = polyfit(pulse_us, azimuth_deg, 1);
azimuthAt1900 = polyval(coefficients, 1900);
baseAngle_deg = azimuth_deg - azimuthAt1900;
coefficients(2) = coefficients(2) - azimuthAt1900;

fitted_deg  = polyval(coefficients, pulse_us);
residual_deg = baseAngle_deg - fitted_deg;
rms_deg = sqrt(mean(residual_deg.^2));

fprintf('\nSlope: %.4f deg per us  (%.2f us per deg)\n', coefficients(1), 1/coefficients(1));
fprintf('Fit RMS: %.3f deg over %d points, max residual %.3f deg\n', ...
    rms_deg, numel(pulse_us), max(abs(residual_deg)));
if rms_deg > 1
    fprintf(['WARNING: residuals over 1 deg. Check that the sweep tag is flat, that nothing ' ...
             'moved between shots, and that each file name matches the pulse actually sent.\n']);
end

out.table = table(fileName, pulse_us, baseAngle_deg, fitted_deg, residual_deg, sizeRms_mm, ...
    'VariableNames', {'photo','pulse_us','baseAngle_deg','fitted_deg','residual_deg','tagSizeRms_mm'});
disp(out.table);

% The firmware and the MATLAB model each take exactly three points.
threePulses = [min(pulse_us) round(mean([min(pulse_us) max(pulse_us)])) max(pulse_us)];
threeAngles = polyval(coefficients, threePulses);
out.coefficients = coefficients;
out.threePulses_us = threePulses;
out.threeAngles_deg = threeAngles;

fprintf('\n--- paste into Rasticrane_XYZ_Test.ino (base row only) ---\n');
fprintf('calibrationPulse_us base row:  {%g,%g,%g}\n', threePulses);
fprintf('calibrationAngle_deg base row: {%.2f,%.2f,%.2f}\n', threeAngles);
fprintf('--- paste into visionConfig.m ---\n');
fprintf('cfg.ik.pulseFit_us(3,:)  = [%g %g %g];\n', threePulses);
fprintf('cfg.ik.angleFit_deg(3,:) = [%.2f %.2f %.2f];\n', threeAngles);
fprintf(['\nThese describe commanded pulse vs measured azimuth only. They are not a ' ...
         'repeatability, backlash, or load test.\n']);

figure('Name','Base pulse vs azimuth');
plot(pulse_us, baseAngle_deg, 'o', pulse_us, fitted_deg, '-');
grid on; xlabel('base pulse (us)'); ylabel('base azimuth (deg, 0 at 1900us)');
legend('measured','linear fit','Location','best');
end
