
clear
global targetX;
global targetY;
global targetR;
global chrLen;
global parNum;

chrLen = 10;
parNum = 2;
roverNum = 10;                  % number of rovers
roverX(1) = 100;                % lead rover start position
roverY(1) = 100;
for ind = 2:roverNum
    roverX(ind)=roverX(ind-1);
    roverY(ind)=roverY(ind-1)+0.5;
end
Xvel(1:roverNum) = 0;           % initial velocities set to zero
Yvel(1:roverNum) = 0;
maxSpeed = 0.2;                 % speed limiter on rovers

targetX=[35,72,140];            % target positions        
targetY=[67,58,125];                      
targetR=[10,10,10];
pathX(:,1)=roverX(:);
pathY(:,1)=roverY(:);
pointer(1:roverNum) = 2;
i=2;

% Creating Initial Chromosomes 
[chrom,coord]=createFirstGen(roverNum,chrLen,parNum);
[F]=evaluate(coord,roverNum,targetX,targetY,targetR);    % evaluate the first generation
[chrom,coord,F]=sort_swap(chrom,coord,F,roverNum);		% sort outcomes according to F score for reproduction (elitism)

for ind=1:50                 % run for 5 waypoints
    [chrom,coord,F]=GA_Search(roverNum,chrom,coord,F);
    waypoints = coord;        % set up temporary matrix to store waypoints
    nextX(:) = waypoints(1,:);
    nextY(:) = waypoints(2,:);  
    waypointX(ind,:) = nextX(:);
    waypointY(ind,:) = nextY(:);
    for c=1:roverNum
        [Xpos,Ypos]=getPath(Xvel(c),Yvel(c),nextX(c),nextY(c),roverX(c),roverY(c),maxSpeed);
        pathEnd(c) = size(Xpos,2)+pointer(c)-1;
        pathX(c,pointer(c):pathEnd(c))=Xpos;
        pathY(c,pointer(c):pathEnd(c))=Ypos;
        roverX(c) = pathX(c,pathEnd(c));
        roverY(c) = pathY(c,pathEnd(c));
        pointer(c) = pathEnd(c);
    end    
end
 figure (1)
 clf
 for i=1:roverNum
     plot(pathX(i,:),pathY(i,:)) 
     hold on
 end
 plot(waypointX,waypointY,'bx') 
 plot(targetX,targetY,'ro')
 axis([0 200 0 200])
