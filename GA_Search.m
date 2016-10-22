% Genetic Algorithms in Matlab (minimising fitness)
function[chromMatrix,coordMatrix]=GA_Search(popSize,chromMatrix,coordMatrix)
% Initial Simulation Data
%Initial Data
generationNum=1;                % number of generations
global parentNum;               % number of parents (number of dimensions)
global chromLen;                % length of chromosomes
global elitePop;
childNum=round(0.8*popSize);    % number of children
elitePop=popSize-childNum;      % number of elite agents
num=(elitePop+1):popSize;       % vector to point at the rest of the population
mutRate=5;                      % mutation rate
bot = 1;                        % floor to start loops from

% create and evaluate next generation of chromosomes
for gen=bot:generationNum,
	kinderArray=crossover2p(popSize,childNum,chromMatrix,chromLen);         % 2 point crossover to create children
	kinderArray=mutate(mutRate,childNum,chromLen,kinderArray);              % mutation to ensure genetic diversity
	kidCoord=decode(kinderArray,childNum,parentNum);                        % decoding children
    [coordMatrix,chromMatrix]=assembleGen(coordMatrix,chromMatrix,num,kidCoord,kinderArray);     % assemble new generation, keeping the elite
end
