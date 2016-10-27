function[chrom]=encode(coord)
global chromLen;
global parentNum;
tempX = coord(1);
tempY = coord(2);
chrom = zeros(1,chromLen);
for ind = 1:chromLen/parentNum
    chrom(1,ind) = floor(tempX/20);
    tempX = (tempX-chrom(1,ind)*20)*10;
    chrom(1,ind+(chromLen/parentNum)) = floor(tempY/20);
    tempY = (tempY-chrom(1,ind+(chromLen/parentNum))*20)*10;
end
