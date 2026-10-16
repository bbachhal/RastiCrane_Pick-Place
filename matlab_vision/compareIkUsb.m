function comparison = compareIkUsb(robotPort,target_mm,cfg)
% Check installed geometry/servo fits, then compare preview pulses. No movement.
if nargin<3, cfg=visionConfig(); end
calibrationReply=char(queryRobotUsb(robotPort,'cal'));
geometryLine=regexp(calibrationReply,'Geometry mm:[^\r\n]*','match','once');
toolLine=regexp(calibrationReply,'Magnet offsets mm:[^\r\n]*','match','once');
geometryValues=regexp(geometryLine,'=(-?[0-9]+\.?[0-9]*)','tokens');
toolValues=regexp(toolLine,'=(-?[0-9]+\.?[0-9]*)','tokens');
reported=cellfun(@(x)str2double(x{1}),[geometryValues toolValues]);
expected=[cfg.ik.upperArm_mm cfg.ik.forearm_mm cfg.ik.shoulderHeight_mm ...
    cfg.ik.shoulderForward_mm cfg.ik.toolForward_mm cfg.ik.toolVertical_mm cfg.ik.toolSideways_mm];
assert(numel(reported)==7 && all(abs(reported-expected)<0.011), ...
    'MATLAB and installed firmware geometry differ. Update them together.');
fits=regexp(calibrationReply,'M([123]) photo fit: ([0-9]+) us -> (-?[0-9]+\.?[0-9]*) deg','tokens');
assert(numel(fits)==9,'Installed firmware calibration format not recognized.');
counts=zeros(1,3);
for k=1:numel(fits)
    motor=str2double(fits{k}{1}); counts(motor)=counts(motor)+1;
    sample=counts(motor);
    assert(sample<=3,'Unexpected servo calibration samples.');
    assert(abs(str2double(fits{k}{2})-cfg.ik.pulseFit_us(motor,sample))<0.01 && ...
        abs(str2double(fits{k}{3})-cfg.ik.angleFit_deg(motor,sample))<0.011, ...
        'MATLAB and firmware servo fits differ.');
end
assert(all(counts==3));
comparison.matlab=rasticraneIkPreview(target_mm,cfg.ik);
comparison.firmwareReply=queryRobotUsb(robotPort,sprintf('ik %.6f %.6f %.6f',target_mm));
pulses=regexp(char(comparison.firmwareReply), ...
    'Proposed pulses M1/M2/base: ([0-9]+) / ([0-9]+) / ([0-9]+) us','tokens','once');
if isempty(pulses)
    comparison.pulsesMatch=false;
    fprintf('Firmware did not propose pulses; inspect rejection above.\n');
    return
end
comparison.firmwarePulses_us=cellfun(@str2double,pulses);
comparison.pulsesMatch=isequal(comparison.firmwarePulses_us,comparison.matlab.pulses_us);
assert(comparison.pulsesMatch,'Preview mismatch. Keep motion disabled and inspect calibration.');
fprintf('PASS: MATLAB and firmware propose identical integer pulses. No motion requested.\n');
end
