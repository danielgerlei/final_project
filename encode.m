function[chrom]=encode(coord)
global chromLen;
global parentNum;
global edgeLength
tempX = coord(1);
tempY = coord(2);
chrom = zeros(1,chromLen);
for ind = 1:chromLen/parentNum
    chrom(1,ind) = floor(tempX/(edgeLength/10));
    tempX = (tempX-chrom(1,ind)*(edgeLength/10))*10;
    chrom(1,ind+(chromLen/parentNum)) = floor(tempY/(edgeLength/10));
    tempY = (tempY-chrom(1,ind+(chromLen/parentNum))*(edgeLength/10))*10;
end
