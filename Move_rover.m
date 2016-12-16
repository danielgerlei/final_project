clc
clear
global Fmax;
global targetXarray;
global targetYarray;
global targetRarray;
global chromLen;
global parentNum;
global NoObstacles;                             
global XObstacle;                       
global YObstacle;
global RObstacle; 
global radioRange;
global generationNum;
global mutRate;
global step;
global KpH;
global KiH;
global KdH;
global edgeLength;

KpH = 8;
KiH = 1;
KdH = -0.75;
radioRange = 40;
roverNum = 10;                  % number of rovers
popSize = 20;
generationNum = 10;             % number of generations run between selecting waypoints
timeLimit = 60;                 % in minutes
step = 0.01;                    % size of timestep
maxSpeed = 0.2;                 % speed limiter on rovers
time = 0;          
parentNum = 2;                  % number of parents
chromLen = 5*parentNum;         % length of chromosomes
mutRate = 10;                   % mutation rate
transmissionFreq = 1;           % every N seconds
transmit = 0;                   % transmission timer
sensorFootprint = 1;            % radius in meters
edgeLength = 200;
timeLimit = timeLimit*60;
timestamps = 0;
stampPointer = 1;

rovers(1) = Rover(100,100,popSize);    % lead rover start position
for r = 2:roverNum                     % line up the rest of the rovers
    rovers(r)=Rover(rovers(r-1).currentX,rovers(r-1).currentY+0.5,popSize);
end

targetXarray=[35,72,140,12];           % target positions        
targetYarray=[67,58,125,55];                      
targetRarray=[2,10,3,5];
Fmax = size(targetXarray,2);

% known obstacles
NoObstacles = 5;                                   % counter for ease of toggleing obstacles
XObstacle = [120 40 40 90 70];                     % position of obstacles
YObstacle = [20 140 80 170 140];
RObstacle = [10 15 5 9 10];                        % radius of obstacles
for r = 1:roverNum
   XObstacle(NoObstacles+r) = rovers(r).currentX;
   YObstacle(NoObstacles+r) = rovers(r).currentY;
   RObstacle(NoObstacles+r) = 1;
end

while (time<timeLimit)
    for r=1:roverNum  
        if (rovers(r).arrived)
            [chromMatrix,coordMatrix]=GA_Search(popSize,rovers(r).chromosomes,rovers(r).decodedChrom);
            ran = ceil(popSize*(rand(1)));  
            waypoint = checkObstacles(coordMatrix,ran,popSize);
            rovers(r).setWaypoint(waypoint);
            rovers(r).chromosomes = chromMatrix;
            rovers(r).decodedChrom = coordMatrix;
        end
        nextTimestamp = rovers(r).getStep(maxSpeed,step,r,time);
        if (nextTimestamp ~= 0)
            [timestamps,stampPointer] = saveStamp(timestamps,stampPointer,nextTimestamp,rovers,r);
        end
        if (transmit == transmissionFreq)
            [neighbours]=rovers(r).findNeighbours(rovers);
            rovers(r).exchangeElite(rovers,neighbours); 
            transmit = 0;
        end
    end
    time = time+step;
    writeTime(time);
    transmit = transmit+step;
end
displayResults(rovers,roverNum,sensorFootprint)