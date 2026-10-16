function cfg = visionConfig()
% Edit this one file after inspecting the numbered detection image.
% Millimeters, degrees; column-vector transforms. No motor commands here.
cfg.tagFamily = 'tag36h11';
cfg.tagSize_mm = 50; % Outer BLACK square, excluding white margin.
cfg.calibrationFile = fullfile('calibration','phoneCalibration.mat');
cfg.outputRoot = 'results';
% Raw MATLAB corner indices in physical BL,BR,TR,TL order of the
% decoded upright printed tag. Determine once from annotatedDetection.png.
% Do NOT copy the OpenCV corner order from the homework PDF.
cfg.cornerOrder = [1 2 3 4]; % e.g. [1 2 3 4] ONLY if the overlay confirms it.
cfg.baseTag.id = 5; % Homework convention; confirm against your physical tag.
cfg.baseTag.centerRobot_mm = [-130 0 4];
cfg.baseTag.yawRobot_deg = 180;
% Robot origin: table below yaw axis. +X is outward at base=1900us,
% +Y left when looking outward along +X, +Z up. Tag local +X: BL->BR.
% A moving arm tag cannot serve as this fixed reference.
cfg.maxPoseFit_px = 3; % Diagnostic rejection threshold, not accuracy guarantee.
cfg.maxTagSizeError_mm = 3; % Review/adjust only after inspecting image quality.
cfg.maxTagShapeError_mm = 3;

% These IDs are explicitly the HW4 convention, not observed new-run IDs.
% Keep heights and rigid attachment offsets NaN until placement is known.
template = struct('name','','id',NaN,'tagHeight_mm',NaN, ...
    'targetOffsetTag_mm',[NaN NaN NaN]);
cfg.objects = repmat(template,1,3);
cfg.objects(1).name = 'inkSurfaceCenter'; cfg.objects(1).id = 2;
cfg.objects(2).name = 'paperCenter'; cfg.objects(2).id = 3;
cfg.objects(3).name = 'checkTag'; cfg.objects(3).id = 1;
cfg.objects(1).tagHeight_mm = 4;   % Tag 2: ink reference
cfg.objects(2).tagHeight_mm = 4;   % Tag 3: paper reference
cfg.objects(3).tagHeight_mm = 10;  % Tag 1: rack reference
% offset = [localX localY vertical] FROM tag CENTER TO desired point.
% A tag on an open hinged lid is not automatically rigidly registered to
% the ink surface. Use a fixed dry tab/bracket, with known CAD offset.
% All tags mapped with Hh in this draft must lie horizontally, parallel to table.

cfg.cad.paperSize_mm = [215.9 279.4];
cfg.cad.paperTop_mm = 0.1; % SINGLE sheet in export, not a stack of paper.
cfg.cad.inkFoamSize_mm = [110 70];
cfg.cad.inkSurface_mm = 5;
cfg.cad.mountSize_mm = [80 80 3];
cfg.cad.rackTop_mm = 36.5; % Bounding-box maximum, NOT disc/pickup height.
cfg.cad.upperArm_mm = 80;
cfg.cad.forearm_mm = 80;
cfg.cad.shoulderForward_mm = 11.991791;
cfg.cad.shoulderAboveArmUnderside_mm = 54.982730;
% If arm underside rests directly on 3mm mount: nominal 57.982730mm.
% This mounting-stack assumption is not substituted into the tested firmware.

% EXACT geometry of installed Rasticrane_XYZ_Test.ino for cross-checking.
% CAD link lengths/forward offset, existing measured magnet/height offsets.
cfg.ik.upperArm_mm = 80; cfg.ik.forearm_mm = 80;
cfg.ik.shoulderForward_mm = 12; cfg.ik.shoulderHeight_mm = 60;
cfg.ik.toolForward_mm = 35; cfg.ik.toolVertical_mm = -6.5;
cfg.ik.toolSideways_mm = 0;
cfg.ik.pulseFit_us = [1500 1900 2000;1500 1900 2000;1300 1900 2500];
cfg.ik.angleFit_deg = [89 61 53;-68 -35 -27;-114.75 -13.01 88.74];
cfg.ik.cartMin_us = [1500 1500 1300];
cfg.ik.cartMax_us = [2000 2000 2500];
cfg.ik.minHeight_mm = 40;
end
