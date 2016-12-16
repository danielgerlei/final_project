function[Frx,Fry]=getRepulsive(X,Y,Kr)                            
global XObstacle;                       
global YObstacle;
global RObstacle;
global edgeLength;
Frx = 0;
Fry = 0;
for n=1:size(XObstacle,2)   
    if (XObstacle(n) ~= X || YObstacle(n) ~= Y)  % omit the current rover
        if (RObstacle(n)>((XObstacle(n)-X)^2 + (YObstacle(n)-Y)^2)^(1/2))
             signX = sign(X-XObstacle(n));
             signY = sign(Y-YObstacle(n));
             if (signX == 0)
                 signX = 1;
             end
             if (signY == 0)
                 signY = 1;
             end
             Frx = Frx + Kr*((1/((((X-XObstacle(n))^2)+((Y-YObstacle(n))^2))^(1/2)))-(1/RObstacle(n)))*signX;
             Fry = Fry + Kr*((1/((((X-XObstacle(n))^2)+((Y-YObstacle(n))^2))^(1/2)))-(1/RObstacle(n)))*signY;
        end
    end
end
if (X > edgeLength*1.1 || X < 0)
    Frx = -sign(X)*Kr;
end
if (Y > edgeLength*1.1 || Y < 0)
    Fry = -sign(Y)*Kr;
end
