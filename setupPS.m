function[rovers,time]=setupPS(rovers,maxSpeed,step,timestamps,stampPointer)
roverNum = size(rovers,2);
time = 0;
startTime = 90;
while (time<startTime)
    for r=1:roverNum  
        if (rovers(r).arrived)
            [startMatrix]=createStartPoints(roverNum);
            waypoint = startMatrix(:,r);        % set up temporary array to store waypoint
            rovers(r).setWaypoint(waypoint);
        end
        nextTimestamp = rovers(r).getStep(maxSpeed,step,r,time);
        if (nextTimestamp ~= 0)
            [timestamps,stampPointer] = saveStamp(timestamps,stampPointer,nextTimestamp,rovers,r);
        end
    end
    time = time+step;
    writeTime(time);
end