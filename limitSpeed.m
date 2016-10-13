function[Xvel,Yvel]=limitSpeed(Xvel,Yvel,maxSpeed)
if (Xvel^2+Yvel^2>maxSpeed^2)
    angle = atand(Yvel/Xvel);
    Yvel=maxSpeed*sin(angle);
    Xvel=maxSpeed*cos(angle);
end