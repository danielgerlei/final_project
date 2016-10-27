function[Xvel,Yvel]=limitSpeed(Xvel,Yvel,maxSpeed)
if (Xvel^2+Yvel^2>maxSpeed^2)
    if(Yvel~=0)
        angle = atand(Yvel/Xvel);
        Yvel=maxSpeed*sin(angle);
        Xvel=maxSpeed*cos(angle);
    else
        if (Xvel>0)
            Xvel = maxSpeed;
        else
            Xvel = -maxSpeed;
        end
    end
end