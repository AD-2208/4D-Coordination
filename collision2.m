function collision_flag = collision2(node, parent, world, dim)

nodeOne = [parent(1),parent(2)];
nodeTwo = [node(1),node(2)];
loop_flag = 0;
world.obsOneX = [0 0 5.5 5.5];
world.obsOneY = [2.5 25 25 6.5];
% obsXTwo = [8 9.5 15 15];
% obsYTwo = [0 10 10 0];
world.obsTwoX = [7.5 9 15.5 15.5];
world.obsTwoY = [0 10.5 10.5 0];
world.obsThreeX = [5 5 25 25];
world.obsThreeY = [23.5 25 25 23.5];
world.obsFourX =[23.5 23.5 25 25];
world.obsFourY = [0 24 24 0];
% If i'th node is outside world boundaries, collision detected with world bounds
for i=1:dim
    if (nodeTwo(i)>world.endcorner(i))||(nodeTwo(i)<world.origincorner(i))
        collision_flag = 1;
    end
end
for i = 1:1:4
    if i < 4
        % Check obs 1
        dt1Obs1 = det([1,1,1;nodeOne(1),nodeTwo(1),world.obsOneX(i);nodeOne(2),nodeTwo(2),world.obsOneY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),world.obsOneX(i+1);nodeOne(2),nodeTwo(2),world.obsOneY(i+1)]);
        dt2Obs1 = det([1,1,1;nodeOne(1),world.obsOneX(i),world.obsOneX(i+1);nodeOne(2),world.obsOneY(i),world.obsOneY(i+1)])*det([1,1,1;nodeTwo(1),world.obsOneX(i),world.obsOneX(i+1);nodeTwo(2),world.obsOneY(i),world.obsOneY(i+1)]);
        % Check obs 2
        dt1Obs2 = det([1,1,1;nodeOne(1),nodeTwo(1),world.obsTwoX(i);nodeOne(2),nodeTwo(2),world.obsTwoY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),world.obsTwoX(i+1);nodeOne(2),nodeTwo(2),world.obsTwoY(i+1)]);
        dt2Obs2 = det([1,1,1;nodeOne(1),world.obsTwoX(i),world.obsTwoX(i+1);nodeOne(2),world.obsTwoY(i),world.obsTwoY(i+1)])*det([1,1,1;nodeTwo(1),world.obsTwoX(i),world.obsTwoX(i+1);nodeTwo(2),world.obsTwoY(i),world.obsTwoY(i+1)]);
        % Check obs 3
        dt1Obs3 = det([1,1,1;nodeOne(1),nodeTwo(1),world.obsThreeX(i);nodeOne(2),nodeTwo(2),world.obsThreeY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),world.obsThreeX(i+1);nodeOne(2),nodeTwo(2),world.obsThreeY(i+1)]);
        dt2Obs3 = det([1,1,1;nodeOne(1),world.obsThreeX(i),world.obsThreeX(i+1);nodeOne(2),world.obsThreeY(i),world.obsThreeY(i+1)])*det([1,1,1;nodeTwo(1),world.obsThreeX(i),world.obsThreeX(i+1);nodeTwo(2),world.obsThreeY(i),world.obsThreeY(i+1)]);
        % Check obs 4
        dt1Obs4 = det([1,1,1;nodeOne(1),nodeTwo(1),world.obsFourX(i);nodeOne(2),nodeTwo(2),world.obsFourY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),world.obsFourX(i+1);nodeOne(2),nodeTwo(2),world.obsFourY(i+1)]);
        dt2Obs4 = det([1,1,1;nodeOne(1),world.obsFourX(i),world.obsFourX(i+1);nodeOne(2),world.obsFourY(i),world.obsFourY(i+1)])*det([1,1,1;nodeTwo(1),world.obsFourX(i),world.obsFourX(i+1);nodeTwo(2),world.obsFourY(i),world.obsFourY(i+1)]);
    else
        % Check obs 1
        dt1Obs1 = det([1,1,1;nodeOne(1),nodeTwo(1),world.obsOneX(i);nodeOne(2),nodeTwo(2),world.obsOneY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),world.obsOneX(1);nodeOne(2),nodeTwo(2),world.obsOneY(1)]);
        dt2Obs1 = det([1,1,1;nodeOne(1),world.obsOneX(i),world.obsOneX(1);nodeOne(2),world.obsOneY(i),world.obsOneY(1)])*det([1,1,1;nodeTwo(1),world.obsOneX(i),world.obsOneX(1);nodeTwo(2),world.obsOneY(i),world.obsOneY(1)]);
        % Check obs 2
        dt1Obs2 = det([1,1,1;nodeOne(1),nodeTwo(1),world.obsTwoX(i);nodeOne(2),nodeTwo(2),world.obsTwoY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),world.obsTwoX(1);nodeOne(2),nodeTwo(2),world.obsTwoY(1)]);
        dt2Obs2 = det([1,1,1;nodeOne(1),world.obsTwoX(i),world.obsTwoX(1);nodeOne(2),world.obsTwoY(i),world.obsTwoY(1)])*det([1,1,1;nodeTwo(1),world.obsTwoX(i),world.obsTwoX(1);nodeTwo(2),world.obsTwoY(i),world.obsTwoY(1)]);
        % Check obs 3
        dt1Obs3 = det([1,1,1;nodeOne(1),nodeTwo(1),world.obsThreeX(i);nodeOne(2),nodeTwo(2),world.obsThreeY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),world.obsThreeX(1);nodeOne(2),nodeTwo(2),world.obsThreeY(1)]);
        dt2Obs3 = det([1,1,1;nodeOne(1),world.obsThreeX(i),world.obsThreeX(1);nodeOne(2),world.obsThreeY(i),world.obsThreeY(1)])*det([1,1,1;nodeTwo(1),world.obsThreeX(i),world.obsThreeX(1);nodeTwo(2),world.obsThreeY(i),world.obsThreeY(1)]);
        % Check obs 4
        dt1Obs4 = det([1,1,1;nodeOne(1),nodeTwo(1),world.obsFourX(i);nodeOne(2),nodeTwo(2),world.obsFourY(i)])*det([1,1,1;nodeOne(1),nodeTwo(1),world.obsFourX(1);nodeOne(2),nodeTwo(2),world.obsFourY(1)]);
        dt2Obs4 = det([1,1,1;nodeOne(1),world.obsFourX(i),world.obsFourX(1);nodeOne(2),world.obsFourY(i),world.obsFourY(1)])*det([1,1,1;nodeTwo(1),world.obsFourX(i),world.obsFourX(1);nodeTwo(2),world.obsFourY(i),world.obsFourY(1)]);
    end

    if(dt1Obs1<=0 && dt2Obs1<=0)
        collision_flag=1;         %If lines intersect in first obstacle
        break
    elseif (dt1Obs2<=0 && dt2Obs2<=0)
        collision_flag=1;         %If lines intersect in second obstacle
        break
    elseif (dt1Obs3<=0 && dt2Obs3<=0)
        collision_flag=1;         %If lines intersect in third obstacle
        break
    elseif (dt1Obs4<=0 && dt2Obs4<=0)
        collision_flag=1;         %If lines intersect in four obstacle
        break
    else
        collision_flag=0;
    end
end
if collision_flag == 0
    for sigma = 0:0.2:1 
        p = sigma*node(1:dim) + (1-sigma)*parent(1:dim); % p is a point on line between current and parent node 
        % check each obstacle
        for i = 1:length(world.ptObsX)
            if (norm( [p(1);p(2)] - [world.ptObsX(i); world.ptObsY(i)]) <= 0.5) % if the point p is within a certain distance of the obstacle, register a collision and break loop
                collision_flag = 1;
                loop_flag = 1;break;
            end
        end
        if loop_flag > 0
            collision_flag = 1;break;
        end
    end
end
world.obsOneX = [0 0 5 5];
world.obsOneY = [3 25 25 7];
world.obsTwoX = [8 9.5 15 15];
world.obsTwoY = [0 10 10 0];
world.obsThreeX = [5 5 25 25];
world.obsThreeY = [24 25 25 24];
world.obsFourX =[24 24 25 25];
world.obsFourY = [0 24 24 0];
end