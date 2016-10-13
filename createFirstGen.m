function[chrom,coord]=createFirstGen(popSize,chrLen,parNum)
% create (popsize) random chromosomes with length (chrlength)
chrom=round(9*rand(popSize,chrLen));			
% translate 10 digit chromosomes into X and Y coordinates
coord=decode(chrom,popSize,parNum);				% decoding chromosomes into coordinates