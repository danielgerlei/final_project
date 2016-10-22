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
global elitePop;

roverNum = 20;                  % number of rovers
popNum = 60;
timeLimit = 30;                 % in minutes
step = 0.2;                     % size of timestep
maxSpeed = 0.2;                 % speed limiter on rovers
time = 0;
chromLen = 10;                  % length of chromosomes
parentNum = 2;                  % number of parents
timeLimit = timeLimit*60;

rovers(1) = Rover(100,100,popNum);     % lead rover start position
for r = 2:roverNum                       % line up the rest of the rovers
    rovers(r)=Rover(rovers(r-1).currentX,rovers(r-1).currentY+0.5,popNum);
end

targetXarray=[35,72,140,12];       % target positions        
targetYarray=[67,58,125,55];                      
targetRarray=[2,10,3,5];
Fmax = size(targetXarray,2);

NoObstacles = 0;                                   % counter for ease of toggleing obstacles
XObstacle = [120 40 40 90 65];                     % position of obstacles
YObstacle = [20 140 80 170 140];
RObstacle = [10 15 5 9 10];                        % radius of obstacles

while (time<timeLimit)
    for r=1:roverNum  
        if (rovers(r).arrived)
            [chromMatrix,coordMatrix]=GA_Search(roverNum,rovers(r).chromosomes,rovers(r).decodedChrom);
            ran = ceil(popNum*(rand(1)));  
            % ran = elitePop+1;   
            waypoint = coordMatrix(:,ran);        % set up temporary matrix to store waypoints
            rovers(r).setWaypoint(waypoint);
            rovers(r).chromosomes = chromMatrix;
            rovers(r).decodedChrom = coordMatrix;
        end
        rovers(r).getStep(maxSpeed,step);
    end
    time = time+step;
    writeTime(time);
end

covered = 0;
figure (1)
clf
 for r=1:roverNum
     [pathX,pathY]=rovers(r).getPathList();
     plot(pathX,pathY) 
     hold on
     [waypoints]=rovers(r).getWaypointsList;
     plot(waypoints(1,:),waypoints(2,:),'bx')
     covered = covered+size(pathX,2);
 end
 ang = 0:0.01:2*pi;
 for n=1:NoObstacles
    xp=RObstacle(n)*cos(ang);
    yp=RObstacle(n)*sin(ang);
    plot(XObstacle(n)+xp,YObstacle(n)+yp,'m');
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
 coverage = covered/((200*chromLen/2)^2)