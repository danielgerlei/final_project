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
global KpH;
global KiH;
global KdH;
global step;
global desiredHeadingArray;
global desiredHeadingPointer;
global headingPointer;
global heading;

KpH = 5;
KiH = 1;
KdH = -0.75;
radioRange = 40;
roverNum = 1;                   % number of rovers
popSize = 20;
generationNum = 10;             % number of generations run between selecting waypoints
timeLimit = 30;                  % in minutes
step = 0.01;                    % size of timestep
maxSpeed = 0.2;                 % speed limiter on rovers
time = 0;          
parentNum = 2;                  % number of parents
chromLen = 5*parentNum;         % length of chromosomes
mutRate = 10;                   % mutation rate
transmissionFreq = 1;           % every N seconds
transmit = 0;                   % transmission timer
sensorFootprint = 1;            % radius in meters
timeLimit = timeLimit*60;
timestamps = 0;
stampPointer = 1;
desiredHeadingArray = 0;
heading = 0;
desiredHeadingPointer = 1;
headingPointer = 1;

rovers(1) = Rover(100,100,popSize);    % lead rover start position
for r = 2:roverNum                     % line up the rest of the rovers
    rovers(r)=Rover(rovers(r-1).currentX,rovers(r-1).currentY+0.5,popSize);
end

targetXarray=[50,20,140,12];           % target positions        
targetYarray=[67,67,125,55];                      
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
target = 1;
Velcounter = 1;
while (time<timeLimit & rovers(1).Xvel<10000 )
    for r=1:roverNum  
        if (rovers(r).arrived)
            waypoint = [targetXarray(target),targetYarray(target)];
            rovers(r).setWaypoint(waypoint);
            target = target+1;
            if target == size(targetXarray,2)+1
                target = 1;
            end
        end
        nextTimestamp = rovers(r).getStep(step,r,time);
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
    Xvelocity(Velcounter) = rovers(1).Xvel;
    Yvelocity(Velcounter) = rovers(1).Yvel;
    Velcounter = Velcounter+1;
    transmit = transmit+step;
end

figure (1)
clf
 for r=1:roverNum
     [pathX,pathY]=rovers(r).getPathList();
     plot(pathX,pathY) 
     hold on
     [waypoints]=rovers(r).getWaypointsList;
     plot(waypoints(1,:),waypoints(2,:),'bx')
 end
 ang = 0:0.01:2*pi;
 for n=1:NoObstacles
    xp=RObstacle(n)*cos(ang);
    yp=RObstacle(n)*sin(ang);
    plot(XObstacle(n)+xp,YObstacle(n)+yp,'k');
end
for n=1:size(targetXarray,2)
    xp=targetRarray(n)*cos(ang);
    yp=targetRarray(n)*sin(ang);
    plot(targetXarray(n)+xp,targetYarray(n)+yp,'r');
end
plot(targetXarray,targetYarray,'rx')
axis([0 200 0 200])

figure (2)
clf
plot((Xvelocity.^2+Yvelocity.^2).^(1/2))

figure (3)
clf
plot(desiredHeadingArray)
hold on
plot(heading,'r')
