function[rovers,time]=setupPS(rovers,maxSpeed,step)
roverNum = size(rovers,2);
time = 0;
startTime = 120;
rand('seed',sum(100*clock));			% randomisation seed initialisation 
while (time<startTime)
    for r=1:roverNum  
        if (rovers(r).arrived)
            [startMatrix]=createStartPoints(roverNum);
            waypoint = startMatrix(:,1);        % set up temporary array to store waypoint
            rovers(r).setWaypoint(waypoint);
        end
        rovers(r).getStep(maxSpeed,step,r);
    end
    time = time+step;
    writeTime(time);
end