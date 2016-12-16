function[coordArray]=decode(chromArray,popSize,parentLen)
global edgeLength;
chromLength = size(chromArray,2);
halfChrom = chromLength/parentLen;
for ind = 1:popSize
	for jin = 1:parentLen,
		jPoint = halfChrom *(jin-1);    
        % multiply the digits of the chromosome by their appropriate decimal power and summing them
		temp1 = (edgeLength/10)*chromArray(ind,1+jPoint)+(edgeLength/100)*chromArray(ind,2+jPoint)+(edgeLength/1000)*chromArray(ind,3+jPoint)+(edgeLength/10000)*chromArray(ind,4+jPoint)+(edgeLength/100000)*chromArray(ind,5+jPoint);
        % treat the fifth digit of the chromosome to represent a power of ten
		%temp2=ceil(chrom(ind,halfchrom + jcon))/4 - 2;
        temp2=0;
        % return 2 by popsize matrix to represent X and y coordinates
		coordArray(jin,ind)=temp1*10^temp2;
	end
end
