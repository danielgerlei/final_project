function[timestamps,stampPointer] = saveStamp(timestamps,stampPointer,nextTimestamp,rovers,r)
timestamps(stampPointer,3) = floor(nextTimestamp/60);
timestamps(stampPointer,4) = round(nextTimestamp-timestamps(stampPointer,3)*60);
timestamps(stampPointer,1) = rovers(r).currentX;
timestamps(stampPointer,2) = rovers(r).currentY;
stampPointer = stampPointer+1;