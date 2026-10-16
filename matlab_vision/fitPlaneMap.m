function planeMap = fitPlaneMap(worldXY_mm,imageUV_px)
% Normalized DLT; H maps world XY to pixels, homogeneous column convention.
assert(size(worldXY_mm,1)>=4 && isequal(size(worldXY_mm),size(imageUV_px)));
assert(all(isfinite(worldXY_mm(:))) && all(isfinite(imageUV_px(:))));
[worldN,worldT] = normalizePoints(worldXY_mm);
[imageN,imageT] = normalizePoints(imageUV_px);
systemMatrix = zeros(2*size(worldN,1),9);
for k=1:size(worldN,1)
    x=worldN(k,1); y=worldN(k,2); u=imageN(k,1); v=imageN(k,2);
    systemMatrix(2*k-1,:)=[-x -y -1 0 0 0 u*x u*y u];
    systemMatrix(2*k,:)=[0 0 0 -x -y -1 v*x v*y v];
end
assert(rank(systemMatrix)>=8,'Degenerate corner correspondences.');
% Full SVD is necessary for the 8x9, four-corner system.
[~,~,rightVectors]=svd(systemMatrix);
normalizedH=reshape(rightVectors(:,end),3,3).';
planeMap=imageT\normalizedH*worldT;
planeMap=planeMap/norm(planeMap,'fro');
assert(rcond(planeMap)>1e-14,'Singular plane mapping.');
end
function [pointsN,transform] = normalizePoints(points)
center=mean(points,1); distance=mean(vecnorm(points-center,2,2));
assert(distance>eps,'Coincident points.'); scale=sqrt(2)/distance;
transform=[scale 0 -scale*center(1);0 scale -scale*center(2);0 0 1];
pointsN=mapPlane(transform,points);
end
