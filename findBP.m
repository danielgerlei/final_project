function[bestPosition] = findBP(rovers,neighbours)
bestPosition = [100,100]; % this creates a force drawing towards the origin until a point of interest is found
strengthBest = getStrength(bestPosition(1),bestPosition(2));
if (neighbours ~=0)
    for n=1:size(neighbours,2)
        roverBestPos = rovers(neighbours(n)).bestPosition;
        roverBestStr = rovers(neighbours(n)).personalBest;
        if (roverBestStr<strengthBest)
            bestPosition = roverBestPos;
        end    
    end
end