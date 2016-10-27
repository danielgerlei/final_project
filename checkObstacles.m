function[waypoint] = checkObstacles(coordMatrix,ran,popSize)
global NoObstacles;
global XObstacle;                       
global YObstacle;
global RObstacle;
waypoint = coordMatrix(:,ran);
[frx,fry] = getRepulsive(waypoint(1),waypoint(2),1);
while frx+fry ~= 0
     ran = ran-1;
     if (ran==0)
         ran = popSize;
     end
     waypoint = coordMatrix(:,ran);
     [frx,fry] = getRepulsive(waypoint(1),waypoint(2),1);
end
end