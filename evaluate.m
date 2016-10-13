function[F]=evaluate(coord,popSize,targetX,targetY,targetR)
for ind=1:popSize
    F(ind)=getStrength(targetX,targetY,targetR,coord(1,ind),coord(2,ind));
end