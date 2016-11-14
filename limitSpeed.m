function[Xvel,Yvel]=limitSpeed(Xvel,Yvel,maxSpeed)
vel = ((Xvel^2+Yvel^2)^(1/2));
if (vel>maxSpeed)
        Xvel = maxSpeed*Xvel/vel;
        Yvel = maxSpeed*Yvel/vel;
end