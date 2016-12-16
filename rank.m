function[F] = rank(coord,F,targetCoverage,targetPointsFound,targetTime,elite)
popSize = size(F,2);
for n = elite+1:popSize
    results = Move_hybrid(coord(1,n)*10^(-5),coord(2,n)*10^(-5),round(coord(3,n)/2),round(coord(2,n)/2)); % limiting imput values
    F(n) = evaluateOptimiser(results,targetCoverage,targetPointsFound,targetTime);      % evaluate the first generation  
end