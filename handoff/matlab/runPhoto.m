% Start here. Opens a photo chooser; never connects to or moves the robot.
projectFolder=fileparts(mfilename('fullpath'));
cd(projectFolder);
[photoName,photoFolder]=uigetfile({'*.jpg;*.jpeg;*.png','Phone photo (JPG or PNG)'});
if ~isequal(photoName,0)
    cfg=visionConfig();
    result=analyzeSetup(fullfile(photoFolder,photoName),cfg);
end
