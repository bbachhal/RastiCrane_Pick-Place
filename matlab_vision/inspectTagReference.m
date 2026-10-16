function cornerOrder = inspectTagReference(referenceImagePath)
% Optional: use the UPRIGHT, FLAT SOURCE IMAGE used to print ONE tag.
% Do not use an oblique/rotated setup photograph as this reference.
% A source tag's lower edge defines local +X; printed top defines local +Y.
referenceImage=imread(referenceImagePath);
[tagIds,corners]=readAprilTag(referenceImage,'tag36h11','DecimationFactor',1);
assert(numel(tagIds)==1,'Provide one upright source tag with its white border.');
corners=corners(:,:,1);
[~,topLeft]=min(sum(corners,2)); [~,bottomRight]=max(sum(corners,2));
[~,bottomLeft]=min(corners(:,1)-corners(:,2));
[~,topRight]=max(corners(:,1)-corners(:,2));
cornerOrder=[bottomLeft bottomRight topRight topLeft];
assert(numel(unique(cornerOrder))==4,'Reference is not an upright square image.');
marked=referenceImage;
for k=1:4
    marked=insertText(marked,corners(k,:),sprintf('raw %d',k), ...
        'FontSize',16,'BoxColor','yellow');
end
figure('Name','Verify against the source used to print your tags'); imshow(marked);
fprintf('Source tag ID %d: cfg.cornerOrder = [%d %d %d %d];\n',tagIds,cornerOrder);
fprintf('Confirm source print orientation; use this convention for mounting yaw/offsets.\n');
end
