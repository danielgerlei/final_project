function[Vfr,Vrr,Vfl,Vrl] = getVoltages(rover,Xvelocity,Yvelocity)
global KpH;
global KiH;
global KdH;

Vmax = 12;
fuzz = 1;   
desiredVel = (Xvelocity^2 + Yvelocity^2)^(1/2);
desiredHeading = asin(Yvelocity/desiredVel);
currentHeading = rover.xi(24);
currentVel = rover.xi(13);
if (Xvelocity < 0)
    desiredHeading = pi*sign(Yvelocity)-desiredHeading;
end
commandHeading = PIDcontroller(desiredHeading,currentHeading,rover,KpH,KiH,KdH);

Vdmax = Vmax;
Vdiff = min (Vdmax, abs(commandHeading));
if commandHeading < -fuzz
    VoltageLeft = Vdiff;
    VoltageRight = 0;
elseif commandHeading > fuzz
    VoltageLeft = 0;
    VoltageRight = Vdiff;
else
    VoltageLeft = Vdiff;
    VoltageRight = Vdiff;
end
commandVel = 0;
if (currentVel<desiredVel)
    commandVel = 12;
end

Vdmax = (Vmax - Vdiff);
Vnew = commandVel;
Vnew = min (Vnew, Vdmax);
Vnew = max (Vnew, 0);

% Add the new voltaages
VoltageLeft = VoltageLeft + Vnew;
VoltageRight = VoltageRight + Vnew;
% while abs(commandHeading) > pi
%     if(commandHeading >= (pi))
%         commandHeading = commandHeading-(2*pi);
%     elseif(commandHeading < -pi)
%         commandHeading = commandHeading+(2*pi);
%     end
% end
% if (abs(currentHeading-commandHeading)<fuzz)
%     VoltageLeft = 12;
%     VoltageRight = 12;
% elseif (currentHeading<commandHeading || ((abs(currentHeading-commandHeading) > pi/2 && (currentHeading>0 && commandHeading<(-pi/2))))) 
%     VoltageRight = 12;
%     VoltageLeft = 0;
% else
%     VoltageLeft = 12;
%     VoltageRight = 0;
% end
Vfr = VoltageRight;
Vrr = VoltageRight;
Vfl = VoltageLeft;
Vrl = VoltageLeft;