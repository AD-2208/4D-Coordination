function plotWorld(world,dim)
% the first element is the north coordinate
% the second element is the south coordinate
if dim ==2 
    N = 100;
    theta = 0:2*pi/N:2*pi;
    hold on
    for i=1:world.NumObstacles
        X = world.radius(i)*sin(theta) + world.cx(i);
        Y = world.radius(i)*cos(theta) + world.cy(i);
        fill(Y,X,'blue');
    end
    
end
end