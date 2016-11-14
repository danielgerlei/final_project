function[bestRover,globalBest,bestPosition] = findBR(rovers,globalBest,bestPosition,bestRover)
roverNum = size(rovers,2);
for r=1:roverNum
    if (rovers(r).personalBest<globalBest)
       globalBest = rovers(r).personalBest;
       bestPosition = rovers(r).bestPosition;
       bestRover = r;
    end
end