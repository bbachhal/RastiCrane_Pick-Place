function result = rasticraneIkPreview(point_mm,model)
% Pure computation. Endpoint screening only; NO trajectory/collision approval.
assert(numel(point_mm)==3 && all(isfinite(point_mm)));
point_mm=reshape(point_mm,1,3);
assert(model.toolSideways_mm==0,'This IK draft requires centered magnet.');
result=struct('target_mm',reshape(point_mm,1,3),'reachable',false, ...
    'angles_deg',[NaN NaN NaN],'pulses_us',[NaN NaN NaN], ...
    'integerResidual_mm',NaN,'endpointInTestRange',false,'reason','');
radial_mm=hypot(point_mm(1),point_mm(2));
horizontal_mm=radial_mm-model.shoulderForward_mm-model.toolForward_mm;
vertical_mm=point_mm(3)-model.shoulderHeight_mm-model.toolVertical_mm;
cosElbow=(horizontal_mm^2+vertical_mm^2-model.upperArm_mm^2-model.forearm_mm^2)/ ...
    (2*model.upperArm_mm*model.forearm_mm);
if radial_mm<1e-9 || horizontal_mm<=0 || abs(cosElbow)>1+1e-10 || hypot(horizontal_mm,vertical_mm)<1e-9
    result.reason='Unreachable or indeterminate IK'; return
end
relativeElbow_deg=-acosd(max(-1,min(1,cosElbow)));
shoulder_deg=atan2d(vertical_mm,horizontal_mm)-atan2d( ...
    model.forearm_mm*sind(relativeElbow_deg), ...
    model.upperArm_mm+model.forearm_mm*cosd(relativeElbow_deg));
result.angles_deg=[shoulder_deg shoulder_deg+relativeElbow_deg atan2d(point_mm(2),point_mm(1))];
assert(norm(rasticraneFk(result.angles_deg,model)-point_mm)<1e-6,'IK/FK mismatch.');
result.reachable=true;
for motor=1:3
    [angleSamples,order]=sort(model.angleFit_deg(motor,:));
    queryAngle_deg=result.angles_deg(motor);
    if queryAngle_deg>=angleSamples(1)-1e-8 && queryAngle_deg<=angleSamples(end)+1e-8
        queryAngle_deg=max(angleSamples(1),min(angleSamples(end),queryAngle_deg));
    end
    result.pulses_us(motor)=interp1(angleSamples,model.pulseFit_us(motor,order), ...
        queryAngle_deg,'linear',NaN);
end
if any(~isfinite(result.pulses_us))
    result.reason='Outside provisional servo calibration; no extrapolation'; return
end
result.pulses_us=round(result.pulses_us);
roundedAngles_deg=zeros(1,3);
for motor=1:3
    roundedAngles_deg(motor)=interp1(model.pulseFit_us(motor,:), ...
        model.angleFit_deg(motor,:),result.pulses_us(motor));
end
roundedPoint_mm=rasticraneFk(roundedAngles_deg,model);
result.integerResidual_mm=norm(roundedPoint_mm-point_mm);
elbowZ_mm=model.shoulderHeight_mm+model.upperArm_mm*sind(roundedAngles_deg(1));
wristZ_mm=roundedPoint_mm(3)-model.toolVertical_mm;
relative_deg=roundedAngles_deg(2)-roundedAngles_deg(1);
result.endpointInTestRange=all(result.pulses_us>=model.cartMin_us & ...
    result.pulses_us<=model.cartMax_us) && ...
    min([elbowZ_mm wristZ_mm roundedPoint_mm(3)])>=model.minHeight_mm && ...
    relative_deg>=-175 && relative_deg<=-5;
if result.endpointInTestRange
    result.reason='Endpoint passes nominal test limits; path/current pose unchecked';
else
    result.reason='Outside current raised Cartesian test region';
end
end
