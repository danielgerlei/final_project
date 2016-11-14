function[XwaypointsMatrix,YwaypointsMatrix] = generateWaypoints(roverNum,sensorFootprint,edgeLength)
rows = 1;
columns = 1;
divider = 2;
remainder = rem(roverNum,divider);
while (remainder>0)
    divider = divider+1;
    remainder = rem(roverNum,divider);
end
rows = divider;
columns = roverNum/divider;
for r = 1:roverNum
    for n = 1:(2*floor((edgeLength/rows)/(sensorFootprint*2)))
        row = floor((r-1)/columns);
        column = rem(r-1,columns);
        nextY = row*(edgeLength/rows)+2*floor((n-1)/2*sensorFootprint);
        nextX = rem(floor(n/2),2)*(edgeLength/columns)+column*(edgeLength/columns);
        if(n<3)
            prevX = nextX+200;   % avoid obstacle on start point
        else
            prevX = XwaypointsMatrix(r,n-2);
        end
        nextX = avoidObstacles(nextX,nextY,prevX);
        XwaypointsMatrix(r,n) = (nextX);
        YwaypointsMatrix(r,n) = (nextY);
    end    
end