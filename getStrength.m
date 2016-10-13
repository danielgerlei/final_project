function[strength]=getStrength(targetX,targetY,targetR,x,y)
strength=0;
s=size(targetX,2);
    for i=1:s
        distance=((targetX(i)-x)^2 + (targetY(i)-y)^2)^(1/2);
        if (distance<targetR(i))
            str=distance/targetR(i);
        else
            str=1;
        end
        strength=strength+str;
    end
