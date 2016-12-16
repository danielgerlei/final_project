function[command]=PIDcontroller(desiredHeading,currentHeading,rover,Kp,Ki,Kd)
% pid Equation
%   Cmd = (LambdaRef - Lambda) * Kp * (KI / s + 1);
%   HdgCmd = Proportional_Hdg + Integral_Hdg + Derivative_Hdg
global step

lastIntegral = rover.PIDvector(1);
lastDot = rover.PIDvector(2);
ErrorSignal = (desiredHeading - currentHeading);
if sign(desiredHeading) ~= sign(currentHeading)
    if ErrorSignal < -pi
         ErrorSignal = ErrorSignal + (2* pi);
    end;
    if ErrorSignal > pi
         ErrorSignal = ErrorSignal - (2* pi);
    end;   
end;
% Integrate to get the new angle to run through the equation
lastIntegral = Euler(lastIntegral, lastDot, step);
% integral getting too large, zero it
if abs(lastIntegral) > 20 % arbitrary saturation value
     lastIntegral = lastIntegral * 0.1;
end
% update lastDot
lastDot = ErrorSignal;
Proportional = ErrorSignal * Kp;
Integral = lastIntegral * Ki;
Derivative = lastDot * Kd;
command = Proportional + Integral + Derivative;

rover.PIDvector(1) = lastIntegral;
rover.PIDvector(2) = lastDot;
        
