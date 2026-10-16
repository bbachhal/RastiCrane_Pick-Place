function reply = queryRobotUsb(robotPort,command)
% Strict allowlist. Never send ready/zero/xyz/jog/manual pulses/off in this draft.
command=strtrim(char(command));
allowed=any(strcmp(command,{'status','where','cal','hwcheck'}));
if startsWith(command,'ik ')
    tokens=strsplit(command);
    numbers=str2double(tokens(2:end));
    allowed=numel(tokens)==4 && all(isfinite(numbers));
end
assert(allowed,'Only status/where/cal/hwcheck/ik x y z are permitted.');
assert(~contains(command,newline) && ~contains(command,char(13)),'One line only.');
flush(robotPort,'input'); writeline(robotPort,command);
reply=""; startTime=tic; completed=false;
while toc(startTime)<8
    if robotPort.NumBytesAvailable>0
        oneLine=readline(robotPort); reply=reply+oneLine+newline;
        fprintf('%s\n',oneLine);
        if any(strcmp(command,{'status','where'}))
            completed=contains(oneLine,'No encoder feedback.');
        elseif strcmp(command,'cal')
            completed=contains(oneLine,'Physical accuracy and repeatability');
        elseif strcmp(command,'hwcheck')
            completed=contains(oneLine,'MATH ONLY A-to-I:');
        else
            completed=contains(oneLine,'PREVIEW ONLY:') || contains(oneLine,'REJECTED:');
        end
        if completed, break; end
    else, pause(0.02); end
end
assert(completed,'No complete expected reply. Inspect firmware/port; no motion was requested.');
end
