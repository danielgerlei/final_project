function[F] = evaluateOptimiser(results,targetCoverage,targetPointsFound,targetTime)
    F = (((targetCoverage-results(1))^2)+((targetPointsFound-results(2))^2)+((targetTime-results(3))^2))^(1/2);

