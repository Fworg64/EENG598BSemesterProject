function [outputArg1,outputArg2] = plotCPs(CPs, ds, Rocks)
%PLOTCPS Summary of this function goes here
%   Detailed explanation goes here
dt = .01;
t = 0:dt:1;
dims = size(CPs);
figure();
hold on;
P0 = [0;0]; %start point
angle1 = 0;
P3 = [0;0];
for index = 1:dims(1)
    P3 = CPs(index, 1:2)' + P3;
    %angle1 = CPs(index  ,3);
    angle2 = CPs(index,3);
    dist1 = ds(index, 1);
    dist2 = ds(index, 2);
P1 = P0 + dist1*[cos(angle1);sin(angle1)];
P2 = P3 - dist2*[cos(angle2);sin(angle2)];

curve = (1 - t).^3 .* P0 ...
        + 3*(1-t).^2 .*t .* P1 ...
        + 3*(1-t).*t.^2 .* P2 ...
        + t.^3 .* P3;
plot(P0(1), P0(2), '*')
plot(curve(1,:), curve(2,:))

P0 = P3;
angle1 = angle2;
end
numRocks= size(Rocks);
for index = 1:numRocks(1)
    Rockxy = [Rocks(index,1) - Rocks(index,3), ...
              Rocks(index,2) - Rocks(index,3), ...
              2*Rocks(index,3),  2*Rocks(index,3)];
   rectangle('Position', Rockxy, 'Curvature', 1); 
end
xlim([0, 6]);
ylim([-5, 5]);

end

