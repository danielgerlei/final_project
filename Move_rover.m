clc
clear
global targetXarray;
global targetYarray;
global targetRarray;
global chromLen;
global parentNum;
global NoObstacles;                             
global XObstacle;                       
global YObstacle;
global RObstacle; 

roverNum = 10;                  % number of rovers
timeLimit = 60;                 % in minutes
step = 0.2;                     % size of timestep
maxSpeed = 0.2;                 % speed limiter on rovers
time = 0;
chromLen = 10;                  % length of chromosomes
parentNum = 2;                  % number of parents
timeLimit = timeLimit*60;

rovers(1) = Rover(100,100);     % lead rover start position
for r = 2:roverNum              % line up the rest of the rovers
    rovers(r)=Rover(rovers(r-1).currentX,rovers(r-1).currentY+0.5);
end

targetXarray=[35,72,140];       % target positions        
targetYarray=[67,58,125];                      
targetRarray=[2,10,3];

NoObstacles = 0;                                    % number of obstacles
XObstacle = [20 40 40 90 65];                       % position of obstacles
YObstacle = [20 40 80 17 40];
RObstacle = [10 15 15 9 10];                        % radius of obstacles

while (time<timeLimit)
    for r=1:roverNum  
        if (rovers(r).arrived)
            [chromMatrix,coordMatrix]=createFirstGen(roverNum);
            [Farray]=zeros(1,roverNum);
            [chromMatrix,coordMatrix,Farray]=GA_Search(roverNum,chromMatrix,coordMatrix,Farray);
            waypoint = coordMatrix(:,1);        % set up temporary matrix to store waypoints
            rovers(r).setWaypoint(waypoint);
        end
        rovers(r).getStep(maxSpeed,step);
    end
    time = time+step;
    writeTime(time);
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
 plot(targetXarray,targetYarray,'ro')
 axis([0 200 0 200])
 
 figure (2)
 clf
 for r=1:roverNum
     fl=rovers(r).getFoundList();
     plot(fl(1,:),fl(2,:),'Marker','o','MarkerEdgeColor',[0.5*r/roverNum 1*r/roverNum 0.3*r/roverNum],'LineStyle','none')
     hold on
 end
 plot(targetXarray,targetYarray,'ro')
 axis([0 200 0 200])