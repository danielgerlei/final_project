
clear
global targetX;
global targetY;
global targetR;
numberOfRovers = 8;
roverX = 100;                   % lead rover start position
roverY = 100;
Xvel = 0;                       % initial velocities
Yvel = 0;
maxSpeed = 0.2;
timelimit = 1000;

targetX=[35,72,140];            % target positions        
targetY=[67,58,125];                      
targetR=[5,5,5];
pathX(1)=roverX;
pathY(1)=roverY;
pointer = 2;
i=2;
for ind=1:10                    % run for 10 waypoints
    [nextX,nextY]=GA_Search(numberOfRovers);
    waypointX(ind) = nextX;
    waypointY(ind) = nextY;
    axis([0 200 0 200])
    dist = ((nextX-roverX)^2 + (nextY-roverY)^2)^(1/2);
    timer = 0;                  % reset timer
    [Xpos,Ypos]=getPath(Xvel,Yvel,nextX,nextY,roverX,roverY,maxSpeed);
    pathEnd = size(Xpos,2)+pointer-1;
    pathX(pointer:pathEnd)=Xpos;
    pathY(pointer:pathEnd)=Ypos;
    roverX = pathX(pathEnd);
    roverY = pathY(pathEnd);
    pointer = pathEnd;
end
 figure (1)
 clf
 plot(pathX,pathY,'g') 
 hold on
 plot(waypointX,waypointY,'bx') 
 plot(targetX,targetY,'ro')
 axis([0 200 0 200])
