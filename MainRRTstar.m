close all
clc

agtCount = 5; % no of runs
dt = 0.1;          % Time step
total_time = 400;  % Total simulation time

dim = 2; %2D
stepSize = 1;
searchRadius = 3; % node search radius
samples = 6000; %node generation
show_output = 1;
start = [1,1; 1,3; 1,5; 1,7; 1,9];
goal = [45,37; 45,39; 45,41; 45,43; 45,45];
totruntime = 0;

[world] = createKnownWorld([50, 50], [0, 0], dim);

xCoords = cell(agtCount, 1);
yCoords = cell(agtCount, 1);
tCoords = cell(agtCount, 1);
safePath = 0;
crashTrue = 0;
attempt_count = 0;

simRun = 0:dt:total_time;
size = length(simRun);
avoidPts = [];


for i = 1:agtCount
    tic;
    if i == 1
        [path,pathDist] = RRTstarFn(dim,stepSize,searchRadius,show_output,samples,start(i,:),goal(i,:), world, i);
        [xPos, yPos, time] = cntrl(path, start(i,:));
        xCoords{i} = xPos;
        yCoords{i} = yPos;
        tCoords{i} = time;
        storePath(i).path = path;
    else
        disp('check')
        safePath = 0;attempt_count = 1;
        while safePath == 0
            [path,pathDist] = RRTstarPP(dim,stepSize,searchRadius,show_output,samples,start(i,:),goal(i,:), world, i);
            [xPos, yPos, time] = cntrl(path, start(i,:));
            xCoords{i} = xPos;
            yCoords{i} = yPos;
            tCoords{i} = time;
            storePath(i).path = path;
            for j = 1:i-1
                if width(xCoords{i}) <= width(xCoords{j})
                    for k = 1:width(xCoords{j})
                        if k <= width(xCoords{i})
                            distance = sqrt((xCoords{i}(k) - xCoords{j}(k))^2 + (yCoords{i}(k) - yCoords{j}(k))^2);
                        else
                            distance = sqrt((xCoords{i}(end) - xCoords{j}(k))^2 +(yCoords{i}(end)- yCoords{j}(k))^2);
                        end
                        if distance < 1
                            disp(distance);
                            if abs(tCoords{i}(k) - tCoords{j}(k)) < 5
                                crashTrue = 1;
                                safePath = 0;
                                break;
                            end
                        end
                    end
                else
                    for k=1:width(xCoords{i})
                        if k <= width(xCoords{j})
                            distance = sqrt((xCoords{i}(k) - xCoords{j}(k))^2 + (yCoords{i}(k) - yCoords{j}(k))^2);
                        else
                            distance = sqrt((xCoords{i}(k) - xCoords{j}(end))^2 +(yCoords{i}(k)- yCoords{j}(end))^2);
                        end
                        if distance < 1
                            disp(distance);
                            if abs(tCoords{i}(k) - tCoords{j}(k)) < 5
                                disp(tCoords{i}(k));disp(tCoords{j}(k))
                                crashTrue = 1;
                                safePath = 0;
                                break;
                            end
                        end
                    end
                end
            end
            if crashTrue == 0
                safePath = 1;
                disp('safe path found')
                break;
            else
                attempt_count = attempt_count+1;
            end
        end
    end
    run_time = toc;
    totruntime = totruntime + run_time;
    strA = ['Time taken for rover ', num2str(i), ' is ', num2str(run_time), ' seconds'];
    disp(strA);
end
str1 = ['The time taken by RRT-Star for ', num2str(agtCount), ' runs is ', num2str(totruntime), ' seconds'];
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp(str1);
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
cntrlLogic(storePath, world, dim);
disp('complete!')

