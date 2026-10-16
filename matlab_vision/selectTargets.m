function out = selectTargets(analysisPath)
% Click the ink-pad center and the paper stamp point, get robot XYZ for both.
% Replaces the loose snippet: it refuses to run on an analysis that never
% produced objects, and it makes you click on the UNDISTORTED image so the
% pixels match the camera model that analyzeSetup fitted.
%
%   selectTargets                      % pick an analysis.mat with a dialog
%   selectTargets('results/2026.../analysis.mat')
%
% Edit these two lines for your physical setup before running.
surfaceHeights_mm = [12.3; 0.1];   % [ink foam top; paper top] above table
targetIds         = [2; 3];        % [ink tag id; paper tag id]
targetName        = {'inkSurfaceCenter'; 'paperStampPoint'};

%% ---- load the analysis -------------------------------------------------
if nargin < 1 || isempty(analysisPath)
    [fileName, fileFolder] = uigetfile(fullfile('results', '*.mat'), ...
        'Pick analysis.mat from a run that completed');
    if isequal(fileName, 0), out = []; return; end
    analysisPath = fullfile(fileFolder, fileName);
end
loaded = load(analysisPath);
assert(isfield(loaded, 'result'), 'That MAT file has no "result" variable.');
result = loaded.result;

%% ---- refuse to continue on an incomplete run ---------------------------
% This is what produced "Dot indexing is not supported...": a rejected run
% leaves result.objects as [], so [result.objects.id] indexes into a double.
if ~strcmp(result.status, 'CALIBRATED_ANALYSIS_PREVIEW_ONLY') || isempty(result.objects)
    fprintf('\nThis run did not reach the calibrated stage.\n');
    fprintf('  status : %s\n', result.status);
    for n = 1:numel(result.notes)
        fprintf('  note   : %s\n', result.notes{n});
    end
    if isfield(result, 'camera') && isfield(result.camera, 'fitRms_px')
        fprintf('  base tag pose fit : %.2f px (limit %.2f px)\n', ...
            result.camera.fitRms_px, result.config.maxPoseFit_px);
    end
    error(['Re-run analyzeSetup on a photo that passes before picking targets. ' ...
           'No targets were computed from this run.']);
end

%% ---- rebuild the undistorted image the camera model was fitted to ------
rawImage = imread(result.sourcePhoto);
if size(rawImage,3) == 1, rawImage = repmat(rawImage,1,1,3); end
stored = load(result.config.calibrationFile);
if isfield(stored,'cameraParams'), intrinsics = stored.cameraParams.Intrinsics;
else,                              intrinsics = stored.intrinsics; end
viewImage = undistortImage(rawImage, intrinsics, 'OutputView', 'same');

%% ---- click the two points ---------------------------------------------
figure('Name','Click 1) ink pad center, then 2) paper stamp point');
imshow(viewImage); hold on; title('Click the INK PAD CENTER');
pickedPixels = zeros(2,2);
for k = 1:2
    if k == 2, title('Click the PAPER STAMP POINT'); end
    [u, v] = ginput(1);
    pickedPixels(k,:) = [u v];
    plot(u, v, 'r+', 'MarkerSize', 14, 'LineWidth', 2);
    text(u+20, v, targetName{k}, 'Color','r', 'FontSize', 12);
end
title('Picked points');

%% ---- pixels -> robot coordinates --------------------------------------
cfg = result.config;
targetXYZ_mm     = zeros(2,3);
targetOffsets_mm = zeros(2,3);

for k = 1:2
    objectIndex = find([result.objects.id] == targetIds(k));
    assert(isscalar(objectIndex), 'Tag id %d is missing or duplicated in the config.', targetIds(k));
    tag = result.objects(objectIndex);
    assert(any(strcmp(tag.status, {'TAG_LOCATED_TARGET_OFFSET_REQUIRED', ...
                                   'TARGET_LOCATED_PREVIEW_ONLY'})), ...
        'Tag %d status is %s, so its pose is not usable.', targetIds(k), tag.status);

    % Map the clicked pixel onto the horizontal plane at that surface height.
    planeMap = heightMap(result.camera, surfaceHeights_mm(k));
    targetXY = mapPlane(inv(planeMap), pickedPixels(k,:));
    targetXYZ_mm(k,:) = [targetXY surfaceHeights_mm(k)];

    % Express the same point in the tag's own frame, so it follows the object
    % when the object is moved and re-photographed.
    angle = tag.tagYaw_deg;
    tagRotation = [cosd(angle) -sind(angle); sind(angle) cosd(angle)];
    offsetXY = tagRotation.' * (targetXY - tag.tagCenterRobot_mm(1:2)).';
    targetOffsets_mm(k,:) = [offsetXY.' surfaceHeights_mm(k) - tag.tagCenterRobot_mm(3)];

    cfg.objects(objectIndex).targetOffsetTag_mm = targetOffsets_mm(k,:);
end

targetTable = table(targetName, targetXYZ_mm(:,1), targetXYZ_mm(:,2), targetXYZ_mm(:,3), ...
    targetOffsets_mm(:,1), targetOffsets_mm(:,2), targetOffsets_mm(:,3), ...
    'VariableNames', {'target','X_mm','Y_mm','Z_mm','offsetX_mm','offsetY_mm','offsetZ_mm'});
disp(targetTable);
fprintf(['These are surface points from one photo. They are NOT magnet commands, ' ...
         'and nothing here has been checked for reachability.\n']);

%% ---- save --------------------------------------------------------------
photoPath     = result.sourcePhoto;
selectionFile = fullfile(result.runFolder, 'targetSelection.mat');
save(selectionFile, 'cfg', 'photoPath', 'pickedPixels', ...
     'targetXYZ_mm', 'targetOffsets_mm', 'targetTable');
zip(fullfile(result.runFolder, 'selectedSetupReview.zip'), ...
    {photoPath, fullfile(result.runFolder,'analysis.mat'), selectionFile});
fprintf('Saved %s\n', selectionFile);

out = struct('cfg',cfg,'pickedPixels',pickedPixels,'targetXYZ_mm',targetXYZ_mm, ...
             'targetOffsets_mm',targetOffsets_mm,'targetTable',targetTable, ...
             'selectionFile',selectionFile);
end
