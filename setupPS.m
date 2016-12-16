function[rovers,time]=setupPS(rovers,step,timestamps,stampPointer)
roverNum = size(rovers,2);
time = 0;
startTime = 180;
while (time<startTime)
    for r=1:roverNum  
        if (rovers(r).arrived)
            [startMatrix]=createStartPoints(1);
            waypoint = startMatrix(:,1);        % set up temporary array to store waypoint
            rovers(r).setWaypoint(waypoint);
        end
        nextTimestamp = rovers(r).getStep(step,r,time);
        if (nextTimestamp ~= 0)
            [timestamps,stampPointer] = saveStamp(timestamps,stampPointer,nextTimestamp,rovers,r);
        end
    end
    time = time+step;
    writeTime(time);
end