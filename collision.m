function collision_flag = collision(node, parent, world, dim)

collision_flag = 0;
loopflag = 0;

% If i'th node is outside world boundaries, collision detected with world bounds
for i=1:dim
    if (node(i)>world.endcorner(i))||(node(i)<world.origincorner(i))
        collision_flag = 1;
    end
end

if collision_flag == 0 && dim ==2
    for sigma = 0:0.2:1 
        p = sigma*node(1:dim) + (1-sigma)*parent(1:dim); % p is a point on line between current and parent node 
        % check each obstacle
        for i=1:world.NumObstacles
            if (norm([p(1);p(2)]-[world.cx(i); world.cy(i)])<=(1+world.radius(i))) % if the point p is within a certain distance of the obstacle, register a collision and break loop
                collision_flag = 1;
                loopflag = 1;break;
            end
        end
        if loopflag > 0
            break;
        end
    end
end
end