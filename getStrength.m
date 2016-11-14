function[strength]=getStrength(x,y)
strength=0;
global targetXarray;
global targetYarray;
global targetRarray;
s=size(targetXarray,2);
for i=1:s
    distance=((targetXarray(i)-x)^2 + (targetYarray(i)-y)^2)^(1/2);
    if (distance<targetRarray(i))
        str=distance/targetRarray(i);
    else
        str=1;
    end
    strength=strength+str;
end
