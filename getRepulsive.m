function[Frx,Fry]=getRepulsive(X,Y,Kr)
global NoObstacles;                             
global XObstacle;                       
global YObstacle;
global RObstacle; 
Frx = 0;
Fry = 0;
for n=1:NoObstacles    
    if (RObstacle(n)<((XObstacle(n)-X)^2 + (YObstacle(n)-Y)^2)^(1/2))
        Frx = Frx + 0;
    else 
        Frx = Frx + Kr*(-(X-XObstacle(n)) / ((((X-XObstacle(n))^2)+((Y-YObstacle(n))^2))^2) + (X-XObstacle(n)) / RObstacle(n)*((((X-XObstacle(n))^2)+((Y-YObstacle(n))^2))^(3/2)) );
    end
      
    if (RObstacle(n)<((XObstacle(n)-X)^2 + (YObstacle(n)-Y)^2)^(1/2))
        Fry = Fry + 0;
    else 
        Fry = Fry + Kr*(-(Y-YObstacle(n)) / ((((X-XObstacle(n))^2)+((Y-YObstacle(n))^2))^2) + (Y-YObstacle(n)) / RObstacle(n)*((((X-XObstacle(n))^2)+((Y-YObstacle(n))^2))^(3/2)) );
    end
end
