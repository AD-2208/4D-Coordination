function [path, pathDist] = RRTstarFn(dim, stepSize, searchRadius, show_output, samples, start_coord, goal_coord, world,rf,ss1,ss2,ss3,ss4,roverNo)

    % Initialize start and end nodes
    start_node = [start_coord, 0, 0, 0]; % x,y,flag for if node connects to end directly, cost, parent
    end_node = [goal_coord, 0, 0, 0];
    tree = start_node; % search space structure

    % Initialize variables
    numPaths = 0;

    % Check if a direct path is possible
    if (norm(start_node(1:dim) - end_node(1:dim)) < stepSize) && (collision2(start_node, end_node, world, dim) == 0)
        disp('CASE 1: Straight line Path found');
        path = [start_node; end_node];
    else
        disp('CASE 2: Finding Path');
        its = 0; % initialize iteration counter
        goalReached = false; % Initialize a flag to check if the goal is reached
        goalThreshold = stepSize; % Define the threshold to be the same as the segment length, or another suitable small value

        while its < samples %&& ~goalReached %%%%%%%% Comment or Uncomment before '&&' to toggle termination at 6k samples and termination as sson as goal is reached
            flag = 0;
            [tree, flag] = extendTree(tree, end_node, stepSize, searchRadius, world, flag, dim, rf, ss1, ss2, ss3, ss4, goal_coord);
            numPaths = numPaths + flag;
            its = its + 1;

            % Check if any node in the tree is within goalThreshold distance to the goal
            for i = 1:size(tree,1)
                if norm(tree(i,1:dim) - goal_coord) <= goalThreshold && collision2(tree(i,:), end_node, world, dim) == 0
                    goalReached = true;
                    break; % Exit the loop if goal is reached
                end
            end
            
%             
%                 % Update visualization
%                 cla; % Clear axes
%                 plotExpandedTree(tree, dim, roverNo); % Plot the expanded tree
%                 drawnow; % Update figure with latest tree
%                 %pause(0.01); % Short pause to allow visualization to update
%             
        end
    end

    % Find path with minimum cost to end_node
    [path, pathDist] = findMinimumPath(tree, end_node, dim);
    if show_output == 1
        plotMap(roverNo);
        %plotExpandedTree(tree, dim, roverNo); %%%%% Comment or Uncomment to visulise the expanded tree
        hold on
        X = path(:,1);
        Y = path(:,2);
        plot(X,Y, 'green-','LineWidth', 0.5);
        % Plot each waypoint as a Black point
        plot(path(:,1), path(:,2), 'black.', 'MarkerSize', 10);
    end
end
