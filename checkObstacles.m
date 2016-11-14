function[waypoint] = checkObstacles(coordMatrix,ran,popSize)
waypoint = coordMatrix(:,ran);
[frx,fry] = getRepulsive(waypoint(1),waypoint(2),1);
while frx^2+fry^2 > 0
     ran = ran-1;
     if (ran==0)
         ran = popSize;
     end
     waypoint = coordMatrix(:,ran);
     [frx,fry] = getRepulsive(waypoint(1),waypoint(2),1);
end
end