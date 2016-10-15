function[chromArray,coordArray]=createFirstGen(popSize)
global chromLen;
global parentNum;
% create (popsize) random chromosomes with length (chrlength)
chromArray=round(9*rand(popSize,chromLen));			
% translate 10 digit chromosomes into X and Y coordinates
coordArray=decode(chromArray,popSize,parentNum);		