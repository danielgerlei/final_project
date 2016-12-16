% Genetic Algorithm based optimiser to determine optimal parameters for the
% searches

clear

% Initial Simulation Data

i = 1;                              % counter
targetCoverage = 100;               % percentage  
targetPointsFound = 200;            % desired result is as high as possible
targetTime = 0;                     % desired result is as low as possible

% Initial Data

genNum = 60;                      % number of generations
popSize = 30;                     % size of population
parNum = 3;                       % number of parents (number of dimensions)
chrLen = 5*parNum;                % length of chromosomes
childNum = round(0.8*popSize);    % number of children
popSize2 = popSize-childNum;      % number of elite agents
num = (popSize2+1):popSize;       % vector to point at the rest of the population
mutRate = 5;                      % mutation rate
bot = 1;                          % floor to start loops from

% Creating Initial Chromosomes 
[chrom,coord]=createFirstGenOptimiser(popSize,chrLen,parNum);

for n = 1:popSize
    results = Move_hybrid(coord(1,n),coord(2,n),coord(2,n));
%     results = Move_rover(coord(1,n),coord(2,n),coord(2,n));
%     results = Move_swarm(coord(1,n),coord(2,n),coord(2,n));
    F(n) = evaluateOptimiser(results,targetCoverage,targetPointsFound,targetTime);      % evaluate the first generation  
end

[chrom,coord,F]=sort_swap(chrom,coord,F,popSize);		% sort outcomes according to F score for reproduction (elitism)

% create and evaluate following generations of chromosomes
for gen=bot:genNum,
	kinder=crossover2p(popSize,childNum,chrom,chrLen);              % 2 point crossover to create children
	kinder=mutate(mutRate,childNum,chrLen,kinder);                  % mutation to ensure genetic diversity
	kidCoord=decode(kinder,childNum,parNum);                        % decoding children
    [coord,chrom]=assembleGen(coord,chrom,num,kidCoord,kinder);     % assemble new generation, keeping the elite
    F=rank(coord,popSize2,childNum,ydesout,y,F);                    % evaluate new generation

    % plot results
    axis([0 10 0 10]) 
    % scale the clour of the plot to make distinguishing between
    % generations plausible
    plot(coord(1,:),coord(2,:),'Marker','o','MarkerEdgeColor',[0.5*gen/genNum 1*gen/genNum 0.3*gen/genNum],'LineStyle','none'),xlabel('c'), ylabel('m'),hold on
    pause(1e-40)
    plot(targetX,targetY,'ro')
 %  plot(q2(1,:),q2(2,:),'wo')
    [chrom,coord,F]=sort_swap(chrom,coord,F,popSize);	% sort new generation
   
end

figure(2)
clf
