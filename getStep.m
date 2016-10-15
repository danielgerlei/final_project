% This code serves to determine a path for a craft
% between two pre-determined points, using potential functions.
% Created by Daniel Gerlei
% Finish date 07/04/2016
% further modifications added to better suit the purposes of my final year
% individual project
% Completed on 
function[]=getStep(Xvel,Yvel,waypoint,X,Y,maxSpeed,rover,step)
%% constants
Xtarget=waypoint(1);
Ytarget=waypoint(2);
Ka = 150;           % attraction constant
Kr = -1.5;          % repulsion constant
stuck = 0;          % velocity under witch the craft is considered stuck
slowSet = 100;      % after bumping into an obstacle, checking for being stuck
slow = slowSet;     % is temporalily halted to avoid a self exciting loop
Kran = 0;           % constant multiplier for the random velocities

%% initiate workspace
NoObstacles = 0;                                    % number of obstacles
XObstacle = [20 40 40 90 65];                       % position of obstacles
YObstacle = [20 40 80 17 40];
RObstacle = [10 15 15 9 10];                        % radius of obstacles

distance = ((Xtarget-X)^2 + (Ytarget-Y)^2)^(1/2);   % distance from target
%% Calculate step
if (distance>0.2)
    % Calculate forces
    Fax = Ka*(X-Xtarget);                           % Attractive forces
    Fay = Ka*(Y-Ytarget);
    % Repulsive forces resulting from any obstacles
    [Frx,Fry]=getRepulsive(X,Y,RObstacle,XObstacle,YObstacle,Kr,NoObstacles);
    Xforce = -(Fax+Frx);
    Yforce = -(Fay+Fry);
    % calculate new position
    Xvel=Euler(Xvel,Xforce,step);   % Euler forward integration to get velocity
    Yvel=Euler(Yvel,Yforce,step);
    [Xvel,Yvel]=limitSpeed(Xvel,Yvel,maxSpeed);
    % if the craft is stuck, generate random velocity
    [Xvel,Yvel,slow]=unStuck(Xvel,Yvel,stuck,slow,slowSet,Kran);
    X = Euler(X,Xvel,step);         % Euler forward integration to get position      
    Y = Euler(Y,Yvel,step);
    % store data 
    coord=[X,Y];
    rover.updatePosition(X,Y);
    rover.savePath();
    strength=getStrength(X,Y);
    rover.saveTarg(coord,strength);
    rover.arrived = false;
else
    rover.arrived = true;
end
 