function[arrayF]=evaluate(coord,popSize)
for ind=1:popSize
    arrayF(ind)=getStrength(coord(1,ind),coord(2,ind));
end