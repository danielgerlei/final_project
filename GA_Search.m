% Genetic Algorithms in Matlab (minimising fitness)
function[m,c]=GA_Search(numberOfRovers)
% Initial Simulation Data
global targetX;
global targetY;
global targetR;

i=1;                            % counter

% Initial Data

genNum=10;                      % number of generations
popSize=numberOfRovers;         % size of population
parNum=2;                       % number of parents (number of dimensions)
chrLen=10;                      % length of chromosomes
childNum=popSize-2;             % number of children
popSize2=popSize-childNum;      % number of elite agents
num=(popSize2+1):popSize;       % vector to point at the rest of the population
mutRate=5;                      % mutation rate
bot = 1;                        % floor to start loops from


% Creating Initial Chromosomes 
[chrom,coord]=createFirstGen(popSize,chrLen,parNum);

[F]=evaluate(coord,popSize,targetX,targetY,targetR);    % evaluate the first generation

[chrom,coord,F]=sort_swap(chrom,coord,F,popSize);		% sort outcomes according to F score for reproduction (elitism)

% create and evaluate following generations of chromosomes
for gen=bot:genNum,
	kinder=crossover2p(popSize,childNum,chrom,chrLen);              % 2 point crossover to create children
	kinder=mutate(mutRate,childNum,chrLen,kinder);                  % mutation to ensure genetic diversity
	kidCoord=decode(kinder,childNum,parNum);                        % decoding children
    [coord,chrom]=assembleGen(coord,chrom,num,kidCoord,kinder);     % assemble new generation, keeping the elite
    [F]=evaluate(coord,popSize,targetX,targetY,targetR);            % evaluate new generation
    [chrom,coord,F]=sort_swap(chrom,coord,F,popSize);               % sort new generation
   
end
m=coord(1,1)		% display best results
c=coord(2,1)
a=F(1)
