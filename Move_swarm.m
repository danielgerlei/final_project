clc
clear
%% Global variables
global targetXarray;
global targetYarray;
global targetRarray;
global chromLen;
global parentNum;
global NoObstacles;                             
global XObstacle;                       
global YObstacle;
global RObstacle; 
global Fmax;
global radioRange;
global KrPersonal;
global KrGlobal;
global step;
global KpH;
global KiH;
global KdH;
global edgeLength;
global desiredHeadingArray;
global desiredHeadingPointer;
global headingPointer;
global heading;

%% Setup
KpH = 7;
KiH = 0;
KdH = 0;
radioRange = 80;
sensorFootprint = 1;            % radius in meters
roverNum = 1;                  % number of rovers
timeLimit = 60;                 % in minutes
step = 0.01;                    % size of timestep
parentNum = 2;                  % number of parents
chromLen = 5*parentNum;         % length of chromosomes
KrPersonal = 15*10^(-1);        % random variables for PS trajectory generation
KrGlobal = 15*10^(-1);
edgeLength = 200;
timeLimit = timeLimit*60;
timestamps = 0;
stampPointer = 1;
desiredHeadingArray = 0;
heading = 0;
desiredHeadingPointer = 1;
headingPointer = 1;

rovers(1) = RoverPS(100,100);   % lead rover start position
for r = 2:roverNum              % line up the rest of the rovers
    rovers(r)=RoverPS(rovers(r-1).currentX,rovers(r-1).currentY+0.5);
end

targetXarray=[35,72,170,12];       % target positions        
targetYarray=[67,58,125,55];                      
targetRarray=[2,10,3,5];
Fmax = size(targetXarray,2);                       % calculate maximum fitness

NoObstacles = 5;                                   % counter for ease of toggleing obstacles
XObstacle = [120 40 40 90 70];                     % position of obstacles
YObstacle = [20 140 80 170 140];
RObstacle = [10 15 5 9 10];                        % radius of obstacles
for r = 1:roverNum
   XObstacle(NoObstacles+r) = rovers(r).currentX;
   YObstacle(NoObstacles+r) = rovers(r).currentY;
   RObstacle(NoObstacles+r) = 1;
end

[rovers,time]=setupPS(rovers,step,timestamps,stampPointer);

%% Main
while (time<timeLimit)
    for r=1:roverNum  
        [strength]=getStrength(rovers(r).currentX,rovers(r).currentY);
        rovers(r).updateBest(strength);
        neighbours = rovers(r).findNeighbours(rovers);
        bestPosition = findBP(rovers,neighbours,rovers(r).bestPosition);
        nextTimestamp = rovers(r).getStepPS(bestPosition,step,r,time);
        if (nextTimestamp ~= 0)
            [timestamps,stampPointer] = saveStamp(timestamps,stampPointer,nextTimestamp,rovers,r);
        end
    end
    time = time+step;
    writeTime(time);
end
displayResults(rovers,roverNum,sensorFootprint)
if roverNum == 1
    figure (4)
    clf
    plot(desiredHeadingArray)
    hold on
    plot(heading,'r')
end