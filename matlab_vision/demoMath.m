function demoMath()
% Synthetic numerical checks only: no image detector, camera, or robot needed.
cfg=visionConfig();
intrinsicMatrix=[2000 0 1280;0 2000 960;0 0 1];
camera.rotation=diag([1 -1 -1]); camera.translation_mm=[0;0;400];
camera.intrinsicMatrix=intrinsicMatrix; camera.centerRobot_mm=[0 0 400];
referenceXY_mm=[-25 -25;25 -25;25 25;-25 25];
referenceHeight_mm=3;
referencePixels=mapPlane(heightMap(camera,referenceHeight_mm),referenceXY_mm);
referenceH=fitPlaneMap(referenceXY_mm,referencePixels);
recovered=recoverCamera(referenceH,intrinsicMatrix,referenceHeight_mm,referenceXY_mm,referencePixels);
assert(norm(recovered.centerRobot_mm-camera.centerRobot_mm)<1e-7);
pixel=mapPlane(heightMap(recovered,0),[100 80]);
assert(norm(pixel-[1780 560])<1e-7,'HW4 table projection failed.');
raisedPixel=mapPlane(heightMap(recovered,30),[200 0]);
wrong=mapPlane(inv(heightMap(recovered,0)),raisedPixel);
correct=mapPlane(inv(heightMap(recovered,30)),raisedPixel);
assert(abs(norm(wrong-[200 0])-30*200/370)<1e-7);
assert(norm(correct-[200 0])<1e-7,'Height correction failed.');
model=cfg.ik; model.upperArm_mm=80; model.forearm_mm=60;
model.shoulderForward_mm=0; model.shoulderHeight_mm=0;
model.toolForward_mm=0; model.toolVertical_mm=0;
hw=rasticraneIkPreview([60 0 90],model);
assert(norm(hw.angles_deg(1:2)-[89.39842073 9.59833882])<1e-6);
unreachableFar=rasticraneIkPreview([150 0 0],model);
unreachableNear=rasticraneIkPreview([10 0 10],model);
assert(~unreachableFar.reachable);
assert(~unreachableNear.reachable);
actual=rasticraneIkPreview([160 25 80],cfg.ik);
assert(isequal(actual.pulses_us,[1969 1968 1955]));
assert(actual.endpointInTestRange);
fprintf('PASS: HW3, HW4, elevated reference plane, parallax correction, firmware IK.\n');
disp(actual);
fprintf('Synthetic checks only. They do not establish physical accuracy.\n');
end
