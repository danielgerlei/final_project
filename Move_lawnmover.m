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

radioRange = 40;
roverNum = 10;                  % number of rovers
popSize = 20;
generationNum = 10;             % number of generations run between selecting waypoints
timeLimit = 60;                 % in minutes
step = 0.2;                     % size of timestep
maxSpeed = 0.2;                 % speed limiter on rovers
time = 0;
parentNum = 2;                  % number of parents
chromLen = 5*parentNum;         % length of chromosomes
mutRate = 10;                   % mutation rate
transmissionFreq = 1;           % every N seconds
transmit = 0;                   % transmission timer
sensorFootprint = 1;            % radius in meters
edgeLength = 200;               % edgelength of the searchspace
timeLimit = timeLimit*60;
timestamps = 0;
stampPointer = 1;

rovers(1) = Rover(100,100,popSize);     % lead rover start position
for r = 2:roverNum                      % line up the rest of the rovers
    rovers(r)=Rover(rovers(r-1).currentX,rovers(r-1).currentY+0.5,popSize);
end

targetXarray=[35,72,140,12];       % target positions        
targetYarray=[67,58,125,55];                      
targetRarray=[2,10,3,5];
Fmax = size(targetXarray,2);

% known obstacles
NoObstacles = 5;                                   % counter for ease of separating rovers and obstacles
XObstacle = [120 41 41 90 70];                     % position of obstacles
YObstacle = [21 141 81 170 141];
RObstacle = [10 15 5 9 10];                        % radius of obstacles

for r = 1:roverNum
   XObstacle(NoObstacles+r) = rovers(r).currentX;
   YObstacle(NoObstacles+r) = rovers(r).currentY;
   RObstacle(NoObstacles+r) = 1;
end

[XwaypointsMatrix,YwaypointsMatrix] = generateWaypoints(roverNum,sensorFootprint,edgeLength);

while (time<timeLimit)
    for r=1:roverNum  
        if (rovers(r).arrived)
            p = rovers(r).waypointsPointer;
            waypoint =[XwaypointsMatrix(r,p),YwaypointsMatrix(r,p)]; 
            rovers(r).setWaypoint(waypoint);
        end
        nextTimestamp = rovers(r).getStep(maxSpeed,step,r,time);
        if (nextTimestamp ~= 0)
            [timestamps,stampPointer] = saveStamp(timestamps,stampPointer,nextTimestamp,rovers,r);
        end
        if (transmit == transmissionFreq)
            [neighbours]=rovers(r).findNeighbours(rovers);
            rovers(r).exchangeElite(rovers,neighbours); 
        end
    end
    time = time+step;
    writeTime(time);
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
for r=1:roverNum
    fl=rovers(r).getFoundList();
    plot(fl(1,:),fl(2,:),'Marker','o','MarkerEdgeColor',[0*r/roverNum 1*r/roverNum 0*r/roverNum],'LineStyle','none')
    hold on
end
plot(targetXarray,targetYarray,'ro')
axis([0 200 0 200])

figure (3)
clf
for r=1:roverNum
    [pathX,pathY]=rovers(r).getPathList();
    plot(pathX,pathY,'k','LineWidth',sensorFootprint*2.5) % set linewidth to simulate sensor coverage
    hold on
end
axis([0 200 0 200])
saveas(gcf,'coverage.bmp','bmp') 
getCoverage();