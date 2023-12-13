function [xPos, yPos, time] = cntrl(path, startPt)

% Simulation parameters
dt = 0.1;          % Time step
total_time = 400;  % Total simulation time
flag = 0;
counter = 1;

% Initialize states
xo = zeros(24,1);
xo(7,1) = startPt(1);
xo(8,1) = startPt(2);
xo(1,1) = 0.75;

simRun = 0:dt:total_time;
size = length(simRun);
xPos = [];yPos = [];
time = [];

% Initialise Current Vals
currentX = 0;
currentY = 0;
currentPsi = 0;
currentVel = 0;

% PID const for u and psi
uKp = 15;   %15 , 3
uKi = 30;   %30 , 13.75
uKd = 0.05;    %0.05 , 0
psiKp = 20;     %20 , 45
psiKi = 0.1;    %0.1 , 3
psiKd = 0.01;   %0.01 , 0.75

% Initialise desired values and errors
psiD = 0;
velD = 0;
eVelCurr = 0;
ePsiMap = 0;
ePsiCurr = 0;
eVelPrev = 0;
ePsiPrev = 0;
eiVel = 0;
eiPsi = 0;
edVel = 0;
eiPsi = 0;
cs_vel = 0;
cs_psi = 0;

psiPrev = 0;
accumulate = 0;
accumulation = 0;
state = 0;
% Waypoint Parameters
wpIdx = 1;
wpX = zeros(length(path),1);
wpY = zeros(length(path),1);

for i = 1:length(path)
    wpX(i) = path(i,1);
    wpY(i) = path(i,2);
end

% Rover variables initialisation
accRadius = 0.2; % Proximity to register reaching wayP

% Run simulation
for t = 0:dt:total_time    
    time = [time;t];
    xPos = [xPos;xo(7,1)];
    yPos = [yPos;xo(8,1)];
    counter = counter + 1;
    
    plot(currentY, currentX);

    resultantVelocity = sqrt((xo(1,1)^2 + (xo(2,1)^2)));
    xo(24,1) = resultantVelocity;

    % Wrap Psi values and assign to current values
    if xo(12) >= pi
        xo(12) = xo(12) - (2*pi);
    elseif xo(12) < -pi
        xo(12) = xo(12) + (2*pi);
    end

    currentX = xo(7);
    currentY = xo(8);
    currentPsi = xo(12);
    currentVel = xo(1);
    
    % LOS Navigation
    dx = wpX(wpIdx) - currentX;
    dy = wpY(wpIdx) - currentY;
    wpEucDist = sqrt((dx)^2 + (dy)^2);
    if wpEucDist <= accRadius
        if wpIdx < length(path)
            wpIdx = wpIdx + 1;
        else
            flag = 1;
        end
    end
    
    if flag == 1
        currentVel = 0;
        currentPsi = 0;
        velD = 0;
        psiD = 0;break;
    end

    psiD = atan2(dy,dx);
    velD = 0.75;

    % Map Psi from [-pi,pi] to [-inf,inf]
    [accumulate,state] = Psi_Mapper_Corrected(psiD, psiPrev, state, dx, dy);

    % Update accumulation variable and psiLast
    accumulation = accumulation + accumulate;
    if accumulation >= 2*pi
        accumulation = accumulation -2*pi;
    end
    psiPrev = psiD;
    
    % Heading
    ePsiCurr = psiD - currentPsi; % psi error
    ePsiMap = PsiMapToPi(ePsiCurr);
    eiPsi = eiPsi + (ePsiMap*dt);
    edPsi = (ePsiMap - ePsiPrev)/dt;
    % Velocity
    eVelCurr = velD - currentVel; % velocity error
    eiVel = eiVel + (eVelCurr*dt);
    edVel = (eVelCurr - eVelPrev)/dt;
    % Control sigs
    cs_vel = (uKp*eVelCurr) + (uKi*eiVel) + (uKd*edVel);
    cs_psi = (psiKp*ePsiMap) + (psiKi*eiPsi) + (psiKd*edPsi);
    ctlSig = [cs_vel;cs_psi];
    % Previous timestep errors
    eVelPrev = eVelCurr; % velocity error for previous timestep's velocity
    ePsiPrev = ePsiMap; % psi error for previous timestep's psi

    % 4th order RK integration
    xdot = fourWheelModel(xo, ctlSig);
    k1 = dt * xdot;          % evaluate derivative k1
    xdot = fourWheelModel((xo+k1)/2, ctlSig);
    k2 = dt * xdot;     % evaluate derivative k2
    xdot = fourWheelModel((xo+k1)/2, ctlSig);
    k3 = dt * xdot;     % evaluate derivative k3
    xdot = fourWheelModel((xo+k3), ctlSig);
    k4 = dt * xdot;		% evaluate derivative k4
    xo = xo + (k1 + 2*k2 + 2*k3 + k4)/6; % averaged output
end
end