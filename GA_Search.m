% Genetic Algorithms in Matlab (minimising fitness)
function[chrom,coord,F]=GA_Search(numberOfRovers,chrom,coord,F)
% Initial Simulation Data
%Initial Data
global targetX;
global targetY;
global targetR;
genNum=5;                       % number of generations
popSize=numberOfRovers;         % size of population
global parNum;                  % number of parents (number of dimensions)
global chrLen;                  % length of chromosomes
childNum=round(0.8*popSize);    % number of children
popSize2=popSize-childNum;      % number of elite agents
num=(popSize2+1):popSize;       % vector to point at the rest of the population
mutRate=5;                      % mutation rate
bot = 1;                        % floor to start loops from

% create and evaluate next generation of chromosomes
for gen=bot:genNum,
	kinder=crossover2p(popSize,childNum,chrom,chrLen);              % 2 point crossover to create children
	kinder=mutate(mutRate,childNum,chrLen,kinder);                  % mutation to ensure genetic diversity
	kidCoord=decode(kinder,childNum,parNum);                        % decoding children
    [coord,chrom]=assembleGen(coord,chrom,num,kidCoord,kinder);     % assemble new generation, keeping the elite
    [F]=evaluate(coord,popSize,targetX,targetY,targetR);            % evaluate new generation
    [chrom,coord,F]=sort_swap(chrom,coord,F,popSize);               % sort new generation 
end
