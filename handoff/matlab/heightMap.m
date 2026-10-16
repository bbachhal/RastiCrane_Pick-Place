function transform = heightMap(camera,height_mm)
assert(isfinite(height_mm) && height_mm<camera.centerRobot_mm(3), ...
    'Height must be finite and below camera.');
transform=camera.intrinsicMatrix*[camera.rotation(:,1:2), ...
    camera.translation_mm+height_mm*camera.rotation(:,3)];
assert(rcond(transform)>1e-14,'Degenerate height plane.');
end
