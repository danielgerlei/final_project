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
%% Setup
radioRange = 20;
sensorFootprint = 1;            % radius in meters
roverNum = 10;                  % number of rovers
timeLimit = 60;                 % in minutes
step = 0.2;                     % size of timestep
maxSpeed = 0.2;                 % speed limiter on rovers
parentNum = 2;                  % number of parents
chromLen = 5*parentNum;         % length of chromosomes
KrPersonal = 0.000005;          % random variables for PS trajectory generation
KrGlobal = 0.000002;
timeLimit = timeLimit*60;
timestamps = 0;
stampPointer = 1;

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

[rovers,time]=setupPS(rovers,maxSpeed,step);
%% Main
while (time<timeLimit)
    for r=1:roverNum  
        [strength]=getStrength(rovers(r).currentX,rovers(r).currentY);
        rovers(r).updateBest(strength);
        neighbours = rovers(r).findNeighbours(rovers);
        bestPosition = findBP(rovers,neighbours);
        nextTimestamp = rovers(r).getStepPS(bestPosition,maxSpeed,step,r,time);
        if (nextTimestamp ~= 0)
            [timestamps,stampPointer] = saveStamp(timestamps,stampPointer,nextTimestamp,rovers,r);
        end
    end
    time = time+step;
    writeTime(time);
end

% Plotting
figure (1)
clf
covered = 0;
 for r=1:roverNum
     [pathX,pathY]=rovers(r).getPathList();
     plot(pathX,pathY) 
     hold on
     covered = covered+size(pathX,2);
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
 axis([-20 220 -20 220])
 figure (2)
 clf
 for r=1:roverNum
     fl=rovers(r).getFoundList();
     plot(fl(1,:),fl(2,:),'Marker','o','MarkerEdgeColor',[0.5*r/roverNum 1*r/roverNum 0.3*r/roverNum],'LineStyle','none')
     hold on
 end
 plot(targetXarray,targetYarray,'ro')
 axis([-20 220 -20 220])
 
figure (3)
clf
for r=1:roverNum
    [pathX,pathY]=rovers(r).getPathList();
    plot(pathX,pathY,'k','LineWidth',sensorFootprint*2) 
    hold on
end
axis([0 200 0 200])
saveas(gcf,'coverage.bmp','bmp') 
getCoverage();
  