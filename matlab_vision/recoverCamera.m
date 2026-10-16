function camera = recoverCamera(planeMap,intrinsicMatrix,referenceHeight_mm,worldXY_mm,imageUV_px)
% HW3/HW4: Hh=K*[r1 r2 t+h*r3]. Decompose reference-plane H.
basis=intrinsicMatrix\planeMap;
scale=2/(norm(basis(:,1))+norm(basis(:,2)));
if median(basis(3,:)*[worldXY_mm.';ones(1,size(worldXY_mm,1))])<0
    scale=-scale;
end
first=scale*basis(:,1); second=scale*basis(:,2);
[left,~,right]=svd([first second cross(first,second)]);
rotation=left*diag([1 1 det(left*right.')])*right.';
translation=scale*basis(:,3)-referenceHeight_mm*rotation(:,3);
camera.rotation=rotation; camera.translation_mm=translation;
camera.intrinsicMatrix=intrinsicMatrix;
camera.centerRobot_mm=(-rotation.'*translation).';
assert(camera.centerRobot_mm(3)>referenceHeight_mm, ...
    'Camera below reference plane: check corner order, frame, calibration.');
cameraPoints=rotation*[worldXY_mm.';referenceHeight_mm*ones(1,size(worldXY_mm,1))]+translation;
assert(all(cameraPoints(3,:)>0),'Reference behind camera.');
predicted=mapPlane(heightMap(camera,referenceHeight_mm),worldXY_mm);
camera.fitRms_px=sqrt(mean(sum((predicted-imageUV_px).^2,2)));
end
