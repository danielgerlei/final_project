% decode function
% inputs: 
%        Chromosomes
%        Population size
%        Number of parents
% output:
%        2 by popsize matrix of X and Y coordinates generated from the
%        input matrix
function[q2]=decode(chrom,popsize,parlen)
length = size(chrom);
chromlength = length(2);
halfchrom = chromlength/2;
for ind = 1:popsize,
	for jin = 1:parlen,
		jcon = halfchrom *(jin-1);                % points to second half of chromosome
        % multiply the digits of the chromosome by their appropriate decimal power and summing them
		temp1=20*chrom(ind,1+jcon)+2*chrom(ind,2+jcon)+0.2*chrom(ind,3+jcon)+0.02*chrom(ind,4+jcon)+0.002*chrom(ind,5+jcon);
        % treat the fifth digit of the chromosome to represent a power of ten
		%temp2=ceil(chrom(ind,halfchrom + jcon))/4 - 2;
        temp2=0;
        % return 2 by popsize matrix to represent X and y coordinates
		q2(jin,ind)=temp1*10^temp2;
	end
end
