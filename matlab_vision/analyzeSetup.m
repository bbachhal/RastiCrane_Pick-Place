function result = analyzeSetup(photoPath,cfg)
% One phone photograph -> detected tags -> height-corrected robot coordinates.
% MATLAB R2024a+ and Computer Vision Toolbox. This function never opens USB.
if nargin<2, cfg=visionConfig(); end
assert(~verLessThan('matlab','24.1'),'Use MATLAB R2024a or newer for this draft.');
assert(isfile(photoPath),'Photo not found: %s',photoPath);
runFolder=fullfile(cfg.outputRoot,char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));
mkdir(runFolder);
result=struct('sourcePhoto',char(photoPath),'matlabVersion',version, ...
    'runFolder',runFolder,'status','DETECTION_ONLY','cameraCalibrated',false, ...
    'notes',{{}},'config',cfg,'objects',[]);
imageRgb=imread(photoPath); % Use identical stored-pixel convention for calibration.
if size(imageRgb,3)==1, imageRgb=repmat(imageRgb,1,1,3); end
assert(size(imageRgb,3)==3,'Use RGB JPG or PNG, not CMYK/HEIC.');
intrinsicMatrix=[];
if isfile(cfg.calibrationFile)
    stored=load(cfg.calibrationFile);
    if isfield(stored,'cameraParams'), intrinsics=stored.cameraParams.Intrinsics;
    elseif isfield(stored,'intrinsics'), intrinsics=stored.intrinsics;
    else, error('Calibration MAT must contain cameraParams or intrinsics.'); end
    assert(isa(intrinsics,'cameraIntrinsics'),'Use a standard perspective camera calibration.');
    assert(isequal(double(intrinsics.ImageSize),double([size(imageRgb,1) size(imageRgb,2)])), ...
        'Photo dimensions differ from calibration. Do not resize to force a match.');
    [imageRgb,newIntrinsics]=undistortImage(imageRgb,intrinsics,'OutputView','same');
    intrinsicMatrix=[newIntrinsics.FocalLength(1) newIntrinsics.Skew newIntrinsics.PrincipalPoint(1); ...
        0 newIntrinsics.FocalLength(2) newIntrinsics.PrincipalPoint(2);0 0 1];
    result.cameraCalibrated=true;
else
    result.notes{end+1}='No camera calibration: detections only, no height-corrected XYZ.';
end
[tagIds,cornerUV_px]=readAprilTag(imageRgb,cfg.tagFamily,'DecimationFactor',1);
tagIds=double(tagIds(:)); result.tagIds=tagIds; result.cornerUV_px=cornerUV_px;
annotated=imageRgb;
cornerRows=zeros(4*numel(tagIds),4);
for k=1:numel(tagIds)
    corners=cornerUV_px(:,:,k);
    annotated=insertShape(annotated,'Polygon',reshape(corners.',1,[]), ...
        'Color','green','LineWidth',3);
    for corner=1:4
        annotated=insertText(annotated,corners(corner,:), ...
            sprintf('%d:%d',tagIds(k),corner),'FontSize',18,'BoxColor','yellow');
        cornerRows(4*(k-1)+corner,:)=[tagIds(k) corner corners(corner,:)];
    end
end
imwrite(annotated,fullfile(runFolder,'annotatedDetection.png'));
writetable(array2table(cornerRows,'VariableNames',{'tagId','matlabCorner','u_px','v_px'}), ...
    fullfile(runFolder,'corners.csv'));
figure('Name','Detected AprilTags: ID : MATLAB corner index'); imshow(annotated);
if isempty(tagIds)
    result.notes{end+1}='No tags decoded. Retake a sharp image with complete borders.';
    finishRun(); return
end
if numel(unique(tagIds))~=numel(tagIds)
    result.notes{end+1}='Duplicate IDs detected. Use unique IDs for this setup.';
    finishRun(); return
end
order=cfg.cornerOrder;
if ~isequal(sort(order),1:4)
    result.notes{end+1}='Set cornerOrder from the numbered overlay: decoded BL,BR,TR,TL.';
    finishRun(); return
end
baseIndex=find(tagIds==cfg.baseTag.id);
if isempty(baseIndex)
    result.notes{end+1}='Fixed reference tag missing. No reconstruction or old-photo fallback.';
    finishRun(); return
end
if any(~isfinite([cfg.baseTag.centerRobot_mm cfg.baseTag.yawRobot_deg]))
    result.notes{end+1}='Set the fixed tag attachment pose relative to the robot yaw axis.';
    finishRun(); return
end
assert(numel(cfg.baseTag.centerRobot_mm)==3 && cfg.tagSize_mm>0);
localCorners_mm=cfg.tagSize_mm/2*[-1 -1;1 -1;1 1;-1 1];
baseRotation=rotation2(cfg.baseTag.yawRobot_deg);
baseWorldXY_mm=localCorners_mm*baseRotation.'+cfg.baseTag.centerRobot_mm(1:2);
basePixels=cornerUV_px(order,:,baseIndex);
referenceH=fitPlaneMap(baseWorldXY_mm,basePixels);
result.referenceH=referenceH;
result.referenceRoundTrip_mm=max(vecnorm(mapPlane(inv(referenceH),basePixels)-baseWorldXY_mm,2,2));
result.notes{end+1}='Four-point round trip checks algebra, not independent physical accuracy.';
if isempty(intrinsicMatrix)
    result.status='REFERENCE_PLANE_ONLY';
    result.notes{end+1}='referenceH is an uncorrected planar approximation; no robot targets exported.';
    finishRun(); return
end
camera=recoverCamera(referenceH,intrinsicMatrix,cfg.baseTag.centerRobot_mm(3),baseWorldXY_mm,basePixels);
result.camera=camera;
if camera.fitRms_px>cfg.maxPoseFit_px
    result.status='REFERENCE_REJECTED';
    result.notes{end+1}='Reference pose fit exceeds threshold; inspect K, size, corners and flatness.';
    finishRun(); return
end
result.tableH=heightMap(camera,0);
objectTemplate=struct('name','','id',NaN,'status','','centerPixel_px',[NaN NaN], ...
    'tagCenterRobot_mm',[NaN NaN NaN],'tagYaw_deg',NaN,'sizeRms_mm',NaN, ...
    'shapeRms_mm',NaN,'targetRobot_mm',[NaN NaN NaN],'emptyMagnetIkAtPoint',[]);
objects=repmat(objectTemplate,1,numel(cfg.objects));
for k=1:numel(cfg.objects)
    spec=cfg.objects(k); objects(k).name=spec.name; objects(k).id=spec.id;
    index=find(tagIds==spec.id);
    if isempty(index), objects(k).status='TAG_MISSING'; continue; end
    pixels=cornerUV_px(order,:,index);
    objects(k).centerPixel_px=mapPlane(fitPlaneMap(localCorners_mm,pixels),[0 0]);
    if ~isfinite(spec.tagHeight_mm)
        objects(k).status='TAG_HEIGHT_REQUIRED'; continue
    end
    objectH=heightMap(camera,spec.tagHeight_mm);
    worldCorners_mm=mapPlane(inv(objectH),pixels);
    centerXY_mm=mean(worldCorners_mm,1);
    centered=worldCorners_mm-centerXY_mm;
    [left,~,right]=svd(localCorners_mm.'*centered);
    tagRotation=right*diag([1 det(right*left.')])*left.';
    fittedCorners_mm=localCorners_mm*tagRotation.'+centerXY_mm;
    sideLengths_mm=vecnorm(worldCorners_mm([2 3 4 1],:)-worldCorners_mm,2,2);
    objects(k).sizeRms_mm=sqrt(mean((sideLengths_mm-cfg.tagSize_mm).^2));
    objects(k).shapeRms_mm=sqrt(mean(sum((fittedCorners_mm-worldCorners_mm).^2,2)));
    objects(k).tagCenterRobot_mm=[centerXY_mm spec.tagHeight_mm];
    objects(k).tagYaw_deg=atan2d(tagRotation(2,1),tagRotation(1,1));
    if objects(k).sizeRms_mm>cfg.maxTagSizeError_mm || objects(k).shapeRms_mm>cfg.maxTagShapeError_mm
        objects(k).status='TAG_GEOMETRY_REJECTED'; continue
    end
    if any(~isfinite(spec.targetOffsetTag_mm))
        objects(k).status='TAG_LOCATED_TARGET_OFFSET_REQUIRED'; continue
    end
    offsetXY_mm=tagRotation*spec.targetOffsetTag_mm(1:2).';
    target_mm=[centerXY_mm+offsetXY_mm.' spec.tagHeight_mm+spec.targetOffsetTag_mm(3)];
    objects(k).targetRobot_mm=target_mm;
    objects(k).emptyMagnetIkAtPoint=rasticraneIkPreview(target_mm,cfg.ik);
    objects(k).status='TARGET_LOCATED_PREVIEW_ONLY';
end
result.objects=objects; result.status='CALIBRATED_ANALYSIS_PREVIEW_ONLY';
targetRows=cell(numel(objects),10);
for k=1:numel(objects)
    obj=objects(k);
    targetRows(k,:)=[{obj.name,obj.id,obj.status},num2cell(obj.tagCenterRobot_mm), ...
        {obj.tagYaw_deg},num2cell(obj.targetRobot_mm)];
end
writetable(cell2table(targetRows,'VariableNames',{'name','tagId','status', ...
    'tagX_mm','tagY_mm','tagZ_mm','tagYaw_deg','targetX_mm','targetY_mm','targetZ_mm'}), ...
    fullfile(runFolder,'targets.csv'));
finishRun();

    function finishRun()
        save(fullfile(runFolder,'analysis.mat'),'result');
        fileId=fopen(fullfile(runFolder,'analysis.json'),'w');
        assert(fileId>=0,'Cannot write analysis.json');
        cleanup=onCleanup(@()fclose(fileId));
        fprintf(fileId,'%s\n',jsonencode(result,'PrettyPrint',true));
        fprintf('\n%s\nResults: %s\n',result.status,runFolder);
        for noteIndex=1:numel(result.notes), fprintf(' - %s\n',result.notes{noteIndex}); end
        if ~isempty(result.objects)
            disp(struct2table(rmfield(result.objects,'emptyMagnetIkAtPoint')));
        end
    end
end
function rotation=rotation2(yaw_deg)
rotation=[cosd(yaw_deg) -sind(yaw_deg);sind(yaw_deg) cosd(yaw_deg)];
end
