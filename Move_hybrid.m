% Move_hybrid(0.00005,0.00001,10,10)
clc
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
global mutRate;
global generationNum;
global step;
global KpH;
global KiH;
global KdH;
global edgeLength;

%% Setup
KpH = 5;
KiH = 1.75;
KdH = -1.35;
radioRange = 40;
sensorFootprint = 1;            % radius in meters
roverNum = 10;                  % number of rovers
timeLimit = 60;                 % time spent exploring the search space in minutes
searchLimit = 3;                % time spent exploring a specific search area (including travel time)
step = 0.01;                    % size of timestep
edgeLength = 200;
parentNum = 2;                  % number of parents
chromLen = 5*parentNum;         % length of chromosomes
mutRate = 10;
generationNum = 10;
transmissionFreq = 1;           % every N seconds
transmit = 0;                   % transmission timer
KrPersonal = 15*10^(-1);        % random variables for PS trajectory generation
KrGlobal = 15*10^(-1);
popSize = 20;
timeLimit = timeLimit*60;     
searchLimit = searchLimit*60;
timestamps = 0;
stampPointer = 1;
searchCounter = 0;

rovers(1) = RoverHybrid(100,100,popSize);   % lead rover start position
for r = 2:roverNum              % line up the rest of the rovers
    rovers(r)=RoverHybrid(rovers(r-1).currentX,rovers(r-1).currentY+0.5,popSize);
end

targetXarray=[35,72,170,12];                       % target positions        
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

[rovers,time] = setupPS(rovers,step,timestamps,stampPointer);
[chromArray,coordArray] = createFirstGen(popSize);
waypoint = findBP(rovers,1:roverNum,rovers(1).bestPosition);
timestamps = [0,0,timeLimit/60,1]; % predefine as high number to avoid errors at output stage if no points are found
% Main
while (time<timeLimit)
    for r=1:roverNum  
        if (searchCounter>searchLimit)
            [chromArray,coordArray]=GA_Search(popSize,chromArray,coordArray);
            ran = ceil(popSize*(rand(1)));  
            waypoint = checkObstacles(coordArray,ran,popSize);
            rovers(r).setWaypoint(waypoint);
        end
        [strength]=getStrength(rovers(r).currentX,rovers(r).currentY);
        rovers(r).updateBest(strength);
        neighbours = rovers(r).findNeighbours(rovers);
        bestPosition = rovers(r).nextWaypoint;
%        bestPosition = findBP(rovers,neighbours);
        nextTimestamp = rovers(r).getStepPS(bestPosition,step,r,time);
        if (nextTimestamp ~= 0)
            [timestamps,stampPointer] = saveStamp(timestamps,stampPointer,nextTimestamp,rovers,r);
        end
    end
    if (transmit == transmissionFreq)
        [neighbours]=rovers(r).findNeighbours(rovers);
        rovers(r).matchWaypoints(rovers,neighbours,r);
        rovers(r).exchangeElite(rovers,neighbours); 
        transmit = 0;
    end
    if (searchCounter>searchLimit)
        searchCounter = 0;
    end
    time = time+step;
    searchCounter = searchCounter+step;
    writeTime(time);
    transmit = transmit+step;
end
displayResults(rovers,roverNum,sensorFootprint) 