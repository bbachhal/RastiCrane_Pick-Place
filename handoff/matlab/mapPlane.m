function result = mapPlane(transform,points)
homogeneous=transform*[points.';ones(1,size(points,1))];
assert(all(abs(homogeneous(3,:))>1e-12),'Point at projective infinity.');
result=(homogeneous(1:2,:)./homogeneous(3,:)).';
end
