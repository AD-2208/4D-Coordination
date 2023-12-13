function [cost] = cost_np2(fromNode,newPoint,world,dim,rf,ss1,ss2,ss3,ss4)
% This function looks up the heuristic cost of travelling to a new point 
% Terrain classes have the following cost: 
    % Steep Slope = 0.9; 
    % Rock Field = 0.5; 
    % Normal Path = 0.1;
% Cost values take into account the maxStep allowed for a move to a new
% node. This ensures that terrain cost will be reasonable in proportion to
% distance cost. 
%---------------------------------------------------------------------%
% Define point 
newPtX = newPoint(1);
newPtY = newPoint(2); 
% Define terrain costs 
cost = 0;
ssCost = 0.9;
rfCost = 0.5; 
normalCost = 0.1; 

% Look up terrain cost of point
if isinterior(ss1,newPtX,newPtY)
    cost = world.step * ssCost;
elseif isinterior(ss2,newPtX,newPtY)
    cost = world.step * ssCost; 
elseif isinterior(ss3,newPtX,newPtY)
    cost = world.step * ssCost; 
elseif isinterior(ss4,newPtX,newPtY)
    cost = world.step * ssCost;
elseif isinterior(rf,newPtX,newPtY)
    cost = world.step * rfCost;
else 
    cost = world.step * normalCost;
end
cost = fromNode(:,dim+2) + cost;
end