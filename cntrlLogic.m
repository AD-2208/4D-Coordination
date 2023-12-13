function cntrlLogic(storePath)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
roverNo = 1;
plotMap(roverNo);
hold on; % Hold on to plot multiple things on the same figure
xlabel('X Position');
ylabel('Y Position');
title('Rover Path Tracking');
grid on;

for agt = 1:length(storePath)
    X = storePath(agt).path(:,1);
    Y = storePath(agt).path(:,2);
    plot(X,Y, 'green-','LineWidth', 0.5);
    % Plot each waypoint as a Black point
    plot(storePath(agt).path(:,1), storePath(agt).path(:,2), 'black.', 'MarkerSize', 10);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
numPaths = length(storePath); % Get the number of paths in storePath

% Simulation parameters
dt = 0.1;          % Time step
total_time = 400;  % Total simulation time
flag = zeros(1,numPaths);
outFlag = 0;
crashCount = 0;

% Initialize states
x = zeros(24,numPaths);
for i = 1:numPaths
    x(1,i) = 0.75;
end
x(7,1) = 4; x(7,2) = 3; x(7,3) = 2; x(7,4) = 1; x(7,5) = 0;
x(8,1) = 1; x(8,2) = 1; x(8,3) = 1; x(8,4) = 1; x(8,5) = 1;

% Initialize cell arrays to store X and Y coordinates and timestamp waypoints
xCoords = cell(numPaths, 1);
yCoords = cell(numPaths, 1);
for j = 1:numPaths
    currentPath = storePath(j).path; % Path matrix for the ith rover path
    xCoords{j} = currentPath(:, 1); % X coordinates (1st column)
    yCoords{j} = currentPath(:, 2); % Y coordinates (2nd column)
end

% Initialise Current Vals
currentX = zeros(1,numPaths);
currentY = zeros(1,numPaths);
currentPsi = zeros(1,numPaths);
currentVel = zeros(1,numPaths);
futureX = zeros(1,numPaths);
futureY = zeros(1,numPaths);

% PID const for u and psi
uKp = 15;   %15 , 3
uKi = 30;   %30 , 13.75
uKd = 0.05;    %0.05 , 0
psiKp = 20;     %20 , 45
psiKi = 0.1;    %0.1 , 3
psiKd = 0.01;   %0.01 , 0.75

% Initialise desired values and errors
psiD = zeros(1,numPaths);
velD = zeros(1,numPaths);
euCurr = zeros(1,numPaths);
ePsiMap = zeros(1,numPaths);
ePsiCurr = zeros(1,numPaths);
euPrev = zeros(1,numPaths);
ePsiPrev = zeros(1,numPaths);
eiu = zeros(1,numPaths);
eiPsi = zeros(1,numPaths);
cs_vel = zeros(1,numPaths);
cs_psi = zeros(1,numPaths);

% Waypoint Parameters
wayPtIdx = ones(1,numPaths);
wpEucDist = zeros(1,numPaths);
for i = 1:numPaths
    wayPtIdx(i) = 1;
    waypX(i) = xCoords{i}(wayPtIdx(i));
    waypY(i) = yCoords{i}(wayPtIdx(i));
end

% Rover variables initialisation
accRadius = 0.1; % Proximity to register reaching wayP
roverMSD = 0.8; % MSD between rovers
dx = zeros(1,numPaths);
dy = zeros(1,numPaths);

% Run simulation
for t = 0:dt:total_time
    if outFlag == 1
        break;
    end
    for i = 1:numPaths
        % Wrap Psi values and assign to current values
        if x(12,i) >= pi
            x(12,i) = x(12,i) - (2*pi);
        elseif x(12,i) < -pi
            x(12,i) = x(12,i) + (2*pi);
        end
        currentX(i) = x(7,i);
        currentY(i) = x(8,i);
        currentPsi(i) = x(12,i);
        currentVel(i) = x(1,i);
        dx(i) = waypX(i) - currentX(i);
        dy(i) = waypY(i) - currentY(i);
        wpEucDist(i) = sqrt((dx(i))^2 + (dy(i))^2);
        if wpEucDist(i) <= accRadius
            if wayPtIdx(i) < length(xCoords{i})
                wayPtIdx(i) = wayPtIdx(i) + 1;
                waypX(i) = xCoords{i}(wayPtIdx(i));
                waypY(i) = yCoords{i}(wayPtIdx(i));
            else
                flag(i) = 1;
            end
        end
        if sum(flag) == 5
            outFlag = 1;break;
        end
        if flag(i) == 1
            x(1,i) = 0;
            x(12,i) = 0;
            currentVel(i) = 0;
            currentPsi(i) = 0;
            velD(i) = 0;
            psiD(i) = 0;
        end
        
        psiD(i) = atan2(dy(i),dx(i));
        
        futureX(i) = currentX(i) + (0.05*cos(currentPsi(i)));
        futureY(i) = currentY(i) + (0.05*sin(currentPsi(i)));
        for j = 1:numPaths
            if j > i && flag(j) == 0
                % Predict the future position of another rover
                otherFutureX = currentX(j) + (0.05*cos(currentPsi(j)));
                otherFutureY = currentY(j) + (0.05*sin(currentPsi(j)));

                % Check for potential collision and prevent it
                if norm([futureX - otherFutureX, futureY - otherFutureY]) > 0.4 && norm([futureX - otherFutureX, futureY - otherFutureY]) <= roverMSD
                    velD(i) = 0.75;velD(j) = 0;
                elseif norm([futureX - otherFutureX, futureY - otherFutureY]) <= 0.4
                    crashCount = crashCount + 1;
                else
                    velD(i) = 0.75;velD(j) = 0.75;
                end
            end
        end
    end

    for i = 1:numPaths
        % Update visualization with the current position of the rovers
        plot(currentX(i), currentY(i), 'red.','MarkerSize', 0.3);
        drawnow; % Update the figure window
    end
    for i = 1:numPaths
        %Current errors
        euCurr(i) = velD(i) - currentVel(i); % velocity error
        ePsiCurr(i) = psiD(i) - currentPsi(i); % psi error
        ePsiMap(i) = PsiMapToPi(ePsiCurr(i));
        %Accumulated Errors
        eiu(i) = eiu(i) + (euCurr(i)*dt);
        eiPsi(i) = eiPsi(i) + (ePsiMap(i)*dt);
        % Control sigs
        cs_vel(i) = (uKp*euCurr(i)) + (uKi*eiu(i)) + (uKd*((euCurr(i) - euPrev(i))/dt));
        cs_psi(i) = (psiKp*ePsiMap(i)) + (psiKi*eiPsi(i)) + (psiKd*((ePsiMap(i) - ePsiPrev(i))/dt));
        ctlSig = [cs_vel(i);cs_psi(i)];
        % Previous timestep errors
        euPrev(i) = euCurr(i); % velocity error for previous timestep's velocity
        ePsiPrev(i) = ePsiMap(i); % psi error for previous timestep's psi

        % 4th order RK integration
        xdot = fourWheelModel(x(:,i), ctlSig);
        k1 = dt * xdot;          % evaluate derivative k1
        xdot = fourWheelModel((x(:,i)+k1)/2, ctlSig);
        k2 = dt * xdot;     % evaluate derivative k2
        xdot = fourWheelModel((x(:,i)+k1)/2, ctlSig);
        k3 = dt * xdot;     % evaluate derivative k3
        xdot = fourWheelModel((x(:,i)+k3), ctlSig);
        k4 = dt * xdot;		% evaluate derivative k4
        x(:,i) = x(:,i) + (k1 + 2*k2 + 2*k3 + k4)/6; % averaged output
    end
    
end
strCrash = ['crash count is ', num2str(crashCount)];
disp(strCrash);
end