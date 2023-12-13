function plotMap(roverNo)
%%% Plotting Environment for paths

% Define map variables
xBoundary = 25;
yBoundary = 25;
% Rover start and end pos
% startXAlpha = 4;    startYAlpha = 1;
% goalXAlpha = 22;    goalYAlpha = 2;
% 
% startXBravo = 3;    startYBravo = 1;
% goalXBravo = 21;    goalYBravo = 3;
% 
% startXCharlie = 2;  startYCharlie = 1;
% goalXCharlie = 20;  goalYCharlie = 4;
% 
% startXDelta = 1;    startYDelta = 1;
% goalXDelta = 18;    goalYDelta = 5;
% 
% startXEcho = 0;     startYEcho = 1;
% goalXEcho = 16;     goalYEcho = 2;
% Define obstacles and terrain objects
obsSafeRadius = 0.2;
obs(1,:) = [16, 22, 20, 18, 21, 21, 20, 19, 17, 22];
obs(2,:) = [12, 14, 12, 15, 12, 15, 14, 13, 14, 17];
obsXOne = [0 0 5 5];
obsYOne = [3 25 25 7];
obsXTwo = [8 10 15 15];
obsYTwo = [0 18 20 0];
obsXThree = [5 5 25 25];
obsYThree = [24 25 25 24];
obsXFour =[24 24 25 25];
obsYFour = [0 24 24 0];
rockFieldX = [15 15 24 24];
rockFieldY = [10 15 18 13];
steepSlopeXOne = [0 0 6 6];
steepSlopeYOne = [2 24 24 6];
steepSlopeXTwo = [7 9 15.5 16 ];
steepSlopeYTwo = [0 19 21 0];
steepSlopeXThree = [6 6 25 25];
steepSlopeYThree = [23.5 25 25 23.5];
steepSlopeXFour = [23 23 25 25];
steepSlopeYFour = [0 23.5 23.5 0];
obstacleOneP = polyshape(obsXOne, obsYOne);
obstacleTwoP = polyshape(obsXTwo, obsYTwo);
obstacleThreeP = polyshape(obsXThree, obsYThree);
obstacleFourP = polyshape(obsXFour, obsYFour);
rockField = polyshape(rockFieldX,rockFieldY);
steepSlopeOne = polyshape(steepSlopeXOne, steepSlopeYOne);
steepSlopeTwo = polyshape(steepSlopeXTwo, steepSlopeYTwo);
steepSlopeThree = polyshape(steepSlopeXThree, steepSlopeYThree);
steepSlopeFour = polyshape(steepSlopeXFour, steepSlopeYFour);
% Plot map
figure;
axis([0 xBoundary 0 yBoundary])
title('Path for Rover', num2str(roverNo))
xlabel('X Position (m)')
ylabel('Y Position (m)')
grid on
hold on
% Plot environment objects
plot(steepSlopeOne, 'FaceColor', 'red', 'FaceAlpha', 0.2)
hold on
plot(steepSlopeTwo, 'FaceColor', 'red', 'FaceAlpha', 0.2)
plot(steepSlopeThree, 'FaceColor', 'red', 'FaceAlpha', 0.2)
plot(steepSlopeFour, 'FaceColor', 'red', 'FaceAlpha', 0.2)
plot(obstacleOneP, 'FaceColor', 'black', 'FaceAlpha', 0.8)
plot(obstacleTwoP, 'FaceColor', 'black', 'FaceAlpha', 0.8)
plot(obstacleThreeP, 'FaceColor', 'black', 'FaceAlpha', 0.8)
plot(obstacleFourP, 'FaceColor', 'black', 'FaceAlpha', 0.8)
plot(rockField, 'FaceColor', 'green', 'FaceAlpha', 0.4)
th = 0:pi/50:2*pi;
xObs1Rad = obsSafeRadius * cos(th) + obs(1,1);
yObs1Rad = obsSafeRadius * sin(th) + obs(2,1);
plot(xObs1Rad,yObs1Rad, 'b');
xObs2Rad = obsSafeRadius * cos(th) + obs(1,2);
yObs2Rad = obsSafeRadius * sin(th) + obs(2,2);
plot(xObs2Rad,yObs2Rad, 'b');
xObs3Rad = obsSafeRadius * cos(th) + obs(1,3);
yObs3Rad = obsSafeRadius * sin(th) + obs(2,3);
plot(xObs3Rad,yObs3Rad, 'b');
xObs4Rad = obsSafeRadius * cos(th) + obs(1,4);
yObs4Rad = obsSafeRadius * sin(th) + obs(2,4);
plot(xObs4Rad,yObs4Rad, 'b');
xObs5Rad = obsSafeRadius * cos(th) + obs(1,5);
yObs5Rad = obsSafeRadius * sin(th) + obs(2,5);
plot(xObs5Rad,yObs5Rad, 'b');
xObs6Rad = obsSafeRadius * cos(th) + obs(1,6);
yObs6Rad = obsSafeRadius * sin(th) + obs(2,6);
plot(xObs6Rad,yObs6Rad, 'b');
xObs7Rad = obsSafeRadius * cos(th) + obs(1,7);
yObs7Rad = obsSafeRadius * sin(th) + obs(2,7);
plot(xObs7Rad,yObs7Rad, 'b');
xObs8Rad = obsSafeRadius * cos(th) + obs(1,8);
yObs8Rad = obsSafeRadius * sin(th) + obs(2,8);
plot(xObs8Rad,yObs8Rad, 'b');
xObs9Rad = obsSafeRadius * cos(th) + obs(1,9);
yObs9Rad = obsSafeRadius * sin(th) + obs(2,9);
plot(xObs9Rad,yObs9Rad, 'b');
xObs10Rad = obsSafeRadius * cos(th) + obs(1,10);
yObs10Rad = obsSafeRadius * sin(th) + obs(2,10);
plot(xObs10Rad,yObs10Rad, 'b');
plot(obs(1,:),obs(2,:), 'o','MarkerSize',5, 'MarkerFaceColor',[0.75, 0, 0.75]);
%---------------------------------------------------------------------%