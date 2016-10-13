function[roverX,roverY] = getStep(roverX,roverY,nextX,nextY,maxSpeed,distance)
diffX = nextX-roverX;
diffY = nextY-roverY;
angle = atand(diffY/diffX);
speed=0.2;
if (distance >= 2)
    roverX = roverX+maxSpeed*cos(angle);
    roverY = roverY+maxSpeed*sin(angle);
else
    roverX = roverX+(distance/2)*maxSpeed*cos(angle);
    roverY = roverY+(distance/2)*maxSpeed*sin(angle); 
    speed = (distance/2)*maxSpeed;
end