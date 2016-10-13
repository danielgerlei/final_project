function[Xvel,Yvel,slow]=unStuck(Xvel,Yvel,stuck,slow,slowSet,Kran)
if (-stuck < Xvel < stuck && -stuck < Yvel < stuck && slow < 1)
    xsign = rand(1);
    if (xsign <=0.5)
        xsign = -1;
    else
        xsign = 1;
    end
    Xvel = rand(1)*Kran*xsign;
    ysign = rand(1);
    if (ysign <=0.5)
        ysign = -1;
    else
        ysign = 1;
    end
    Yvel = rand(1)*Kran*ysign;
    slow = slowSet;
end