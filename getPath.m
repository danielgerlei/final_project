% This code serves to determine a path for a craft
% between two pre-determined points, using potential functions.
% Created by Daniel Gerlei
% Finish date 07/04/2016
function[Xpos,Ypos]=getPath(Xvel,Yvel,Xtarget,Ytarget,X,Y,maxSpeed)
%% constants
Ka = 100;          % attraction constant
Kr = -1.5;       % repulsion constant
Kv = 2.5;        % velocity damping constant
timelimit = 1000; % in seconds
step = 0.2;      % timestep
stuck = 0;    % velocity under witch the craft is considered stuck
slowset = 100;   % after bumping into an obstacle, checking for being stuck
slow = slowset;  % is temporalily halted to avoid a self exciting loop
Kran = 0;       % constant multiplier for the random velocities
%% variables
time = 1;        % start of the simulation
i=1;             % pointer for the final trajectory vectors
%% initiate workspace
NoObstacles = 0;                            % number of obstacles
Xobstacle = [20 40 40 90 65];               % position of obstacles
Yobstacle = [20 40 80 17 40];
Robstacle = [10 15 15 9 10];                % radius of obstacles

distance = ((Xtarget-X)^2 + (Ytarget-Y)^2)^(1/2); % distance from target
%% Calculate path
while (time<timelimit && distance>0.2)
    % reseting forces to avoid unwanted accumulation
    Frx = 0;
    Fry = 0;
    % Calculate forces
    Fax = Ka*(X-Xtarget);  % Attractive forces
    Fay = Ka*(Y-Ytarget);
    for n=1:NoObstacles    % Repulsive forces resulting from any obstacles
        if (Robstacle(n)<((Xobstacle(n)-X)^2 + (Yobstacle(n)-Y)^2)^(1/2))
            Frx = Frx + 0;
        else 
            Frx = Frx + Kr*(-(X-Xobstacle(n)) / ((((X-Xobstacle(n))^2)+((Y-Yobstacle(n))^2))^2) + (X-Xobstacle(n)) / Robstacle(n)*((((X-Xobstacle(n))^2)+((Y-Yobstacle(n))^2))^(3/2)) );
        end
        
        if (Robstacle(n)<((Xobstacle(n)-X)^2 + (Yobstacle(n)-Y)^2)^(1/2))
            Fry = Fry + 0;
        else 
            Fry = Fry + Kr*(-(Y-Yobstacle(n)) / ((((X-Xobstacle(n))^2)+((Y-Yobstacle(n))^2))^2) + (Y-Yobstacle(n)) / Robstacle(n)*((((X-Xobstacle(n))^2)+((Y-Yobstacle(n))^2))^(3/2)) );
        end
    end
    Xforce = -(Fax+Frx);%-Kv*Xvel;    % velocity term substracted,
    Yforce = -(Fay+Fry);%-Kv*Yvel;    % to introduce damping
    % calculate new position
    Xvel = Xvel + Xforce*step; % Euler forward integration to get velocity   
    Yvel = Yvel + Yforce*step; 
    if (Xvel^2+Yvel^2>maxSpeed^2)
        angle = atand(Yvel/Xvel);
        Yvel=maxSpeed*sin(angle);
        Xvel=maxSpeed*cos(angle);
    end
    % if the craft is stuck, generate random velocity
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
        slow = slowset;
    end
    X = X + Xvel*step;         % Euler forward integration to get position      
    Y = Y + Yvel*step;
%     storeStrength(X,Y,)
    % store data 
    Xpos(i) = X;                 
    Ypos(i) = Y;
    % update time and distance to check against end condition
    distance = ((Xtarget-X)^2 + (Ytarget-Y)^2)^(1/2);
    time = time+step;
    i=i+1;
    slow = slow-1;
end  