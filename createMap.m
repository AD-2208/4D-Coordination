function [world,rockField,steepSlopeOne,steepSlopeTwo,steepSlopeThree,steepSlopeFour] = createMap()
%%% Plotting Environment for paths

% Define map variables
% xBoundary = 25;
% yBoundary = 25;
% % Rover start and end pos
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
% World Struct
world.origincorner = [0,0,0];
world.endcorner = [25,25,25];
world.obsOneX = [0 0 5 5];
world.obsOneY = [3 25 25 7];
world.obsTwoX = [8 10 15 15];
world.obsTwoY = [0 18 20 0];
world.obsThreeX = [5 5 25 25];
world.obsThreeY = [24 25 25 24];
world.obsFourX =[24 24 25 25];
world.obsFourY = [0 24 24 0];
world.step = 1;
world.ptObsX = obs(1,:);
world.ptObsY = obs(2,:);
world.obsTh = obsSafeRadius;