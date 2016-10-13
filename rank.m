function[F]=rank(coord,popSize2,childNum,ydesout,y,F)   
for inde=1:childNum,
		ind = inde + popSize2;
	  	m=coord(1,ind);
        c=coord(2,ind);
        i=1;
        for x = -10:0.2:10,						% application simulation y=mx+c (function for filter to go here)
            y(i)=m*x+c;
            i=i+1;
        end
  		F(ind) = sum(abs(y-ydesout))^2;			% cost function
end