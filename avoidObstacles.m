function[nextX] = avoidObstacles(nextX,nextY,prevX)
[Frx,Fry]=getRepulsive(nextX,nextY,1);
while ((Frx^2+Fry^2)>0)
    nextX = nextX + sign(prevX-nextX)*0.04;
    [Frx,Fry]=getRepulsive(nextX,nextY,1);
end