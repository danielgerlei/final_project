function[F,y]=rankGen1(coord,popSize,ydesout) 
for ind=1:popSize,
 	  m=coord(1,ind);                           % represent gradient with coordinate X
      c=coord(2,ind);                           % represent displacement with coordinate Y
      i=1;
      for x = -10:0.2:10,						% application simulation y=mx+c
      	y(i)=m*x+c;
      	i=i+1;
      end
      F(ind) = sum(abs(y-ydesout))^2;           % evaluate outcomes against desired output
end