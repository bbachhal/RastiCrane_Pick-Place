function robotPort = openRobotUsb(portName)
% Explicit opt-in: opening USB MAY RESET the ESP32 and engage startup home.
% Support/clear the arm first; close Arduino Serial Monitor first.
% Read README before using. This draft sends only diagnostic queries.
robotPort=serialport(portName,115200,'Timeout',2);
configureTerminator(robotPort,'LF');
pause(4); % Existing firmware staggers startup; allow boot output to finish.
if robotPort.NumBytesAvailable>0
    bootText=char(read(robotPort,robotPort.NumBytesAvailable,'uint8'));
    fprintf('%s',bootText);
end
fprintf('USB open. Diagnostic commands only. No motion completion is inferred.\n');
end
