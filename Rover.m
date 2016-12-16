
classdef Rover < handle
    properties
        foundList
        foundPointer
        currentX
        currentY
        Xvel
        Yvel
        pathXarray
        pathYarray
        pathPointer
        waypointsList
        waypointsPointer
        arrived
        nextWaypoint
        chromosomes
        decodedChrom
        velVector
        xi
        xo
        PIDvector
% xi and xo are 24 member vectors with the following structure:
% 1 Left Front Wheel Current
% 2 Left Front Wheel Rotational Spee
% 3 Left Front Wheel Torque
% 4 Left Rear Wheel Current
% 5 Left Rear Wheel Rotational Spee
% 6 Left Rear Wheel Torque
% 7 Right Front Wheel Current
% 8 Right Front Wheel Rotational Spe
% 9 Right Front Wheel Torque
% 10 Right Rear Wheel Current
% 11 Right Rear Wheel Rotational Spe
% 12 Right Rear Wheel Torque
% 13 u - DoF(1) - Surge Velocity
% 14 v - DoF(2) - Sway Velocity
% 15 w - DoF(3) - Heave Velocity
% 16 p - DoF(4) - Roll Velocity
% 17 q - DoF(5) - Pitch Velocity
% 18 r - DoF(6) - Yaw Velocity
% 19 x - DoF(1) - Surge position
% 20 y - DoF(2) - Sway position
% 21 z - DoF(3) - Heave position
% 22 - DoF(4) - Roll position
% 23 - DoF(5) - Pitch position
% 24 - DoF(6) - Yaw position
    end
    
    methods
        function[this]=Rover(x,y,popSize)
            global Fmax;
            this.pathPointer = 2;
            this.foundPointer = 1;
            this.waypointsPointer = 1;
            this.foundList = [100000000;10000000000;Fmax];
            this.currentX = x;
            this.currentY = y;
            this.Xvel = 0;
            this.Yvel = 0;
            this.velVector = [this.Xvel,this.Yvel];
            this.pathXarray = x;
            this.pathYarray = y;
            this.arrived = true;
            this.nextWaypoint = [x,y];
            this.xiPack();
            this.xo = this.xi;
            [this.chromosomes,this.decodedChrom] = createFirstGen(popSize);
            this.PIDvector = [0,0];
        end
        
        function[timestamp]=getStep(this,maxSpeed,step,roverPointer,time)
            global NoObstacles;                             
            global XObstacle;                       
            global YObstacle;
            
            timestamp = 0;
            Xtarget = this.nextWaypoint(1);
            Ytarget = this.nextWaypoint(2);
            X = this.currentX;
            Y = this.currentY;
            Xvelocity = this.Xvel;
            Yvelocity = this.Yvel;
            Ka = 100;           % attraction constant
            Kr = -3000000;     % repulsion constant
            stuck = 0;          % velocity under witch the craft is considered stuck
            slowSet = 100;      % after bumping into an obstacle, checking for being stuck
            slow = slowSet;     % is temporalily halted to avoid a self exciting loop
            Kran = 0;           % constant multiplier for the random velocities

            distance = ((Xtarget-X)^2 + (Ytarget-Y)^2)^(1/2);   % distance from target
            if (distance>0.2)
                % Calculate forces
                Fax = Ka*(X-Xtarget);                           % Attractive forces
                Fay = Ka*(Y-Ytarget);
                % Repulsive forces resulting from any obstacles
                [Frx,Fry]=getRepulsive(X,Y,Kr);
                Xforce = -(Fax+Frx);
                Yforce = -(Fay+Fry);
                % calculate new position
                Xvelocity=Euler(Xvelocity,Xforce,step);   % Euler forward integration to get velocity
                Yvelocity=Euler(Yvelocity,Yforce,step);
%                 [Xvelocity,Yvelocity]=limitSpeed(Xvelocity,Yvelocity,maxSpeed);
                % if the craft is stuck, generate random velocity
                [Xvelocity,Yvelocity,slow] = unStuck(Xvelocity,Yvelocity,stuck,slow,slowSet,Kran);
                [Vfr,Vrr,Vfl,Vrl] = getVoltages(this,Xvelocity,Yvelocity);
                on = 1;
                if (on == 1)
                    [xdot, this.xo] = full_mdl_motors([Vfr,Vrr,Vfl,Vrl],this.xi,0,0,0,0,step); 
                    this.xi = Euler(this.xo,xdot,step);
                    this.xiUnpack();
                else
                    this.Xvel = Xvelocity;
                    this.Yvel = Yvelocity;
                    X = Euler(X,Xvelocity,step);
                    Y = Euler(Y,Yvelocity,step);
                    this.currentX = X;
                    this.currentY = Y;
                end
                XObstacle(NoObstacles+roverPointer) = this.currentX;
                YObstacle(NoObstacles+roverPointer) = this.currentY;
                this.savePath();
                strength=getStrength(X,Y);
                timestamp = this.saveTarg([this.currentX,this.currentY],strength,time);
                this.arrived = false;
            else
                this.arrived = true;
            end
        end
        
        function[]=xiUnpack(this)
            this.currentX = this.xi(19);
            this.currentY = this.xi(20);
            this.Xvel = this.xi(13)*cos(this.xi(24));
            this.Yvel = this.xi(13)*sin(this.xi(24));
            this.velVector = [this.Xvel,this.Yvel];
        end
        
        function[]=xiPack(this)
            this.xi = zeros(1,24);
            this.xi(19) = this.currentX;
            this.xi(20) = this.currentY;
            this.xi(13) = (this.Xvel^2 + this.Yvel^2)^(1/2);
            if (this.Xvel == 0)
                this.xi(24) = sign(this.Yvel)*(pi/2);
            else
                this.xi(24) = atan(this.Yvel/this.Xvel);
            end     
            if (this.Yvel == 0)
                this.xi(24) = (sign(this.Xvel)-1)*pi;
            end
        end
        
        function[]=setWaypoint(this,waypoint)
            this.waypointsList(1,this.waypointsPointer) = waypoint(1);
            this.waypointsList(2,this.waypointsPointer) = waypoint(2);
            this.nextWaypoint(1) = waypoint(1);
            this.nextWaypoint(2) = waypoint(2);
            this.waypointsPointer = this.waypointsPointer+1;            
        end
        
        function[waypoints]=getWaypointsList(this)
            waypoints = this.waypointsList;
        end
        
        function[]=updatePosition(this,x,y)
           this.currentX=x;
           this.currentY=y;
        end
        
        function[]=savePath(this)
            this.pathXarray(this.pathPointer) = this.currentX;
            this.pathYarray(this.pathPointer) = this.currentY;
            this.pathPointer = this.pathPointer+1;
        end
        
        function[pathX,pathY]=getPathList(this)
            pathX=this.pathXarray;
            pathY=this.pathYarray;
        end
                
        function[]=setVel(this,Xvel,Yvel)
            this.Xvel=Xvel;
            this.Yvel=Yvel;
        end
            
        function[timestamp]=saveTarg(this,coord,F,time)
            global Fmax;
            global elitePop;
            timestamp = 0;
            if (F<Fmax)
                exist=false;
                for i=1:size(this.foundList,2)
                    if (((this.foundList(1,i)-coord(1))^2 + (this.foundList(2,i)-coord(2))^2)^(1/2) < 3)
                        exist=true;
                        if (this.foundList(3,i) <= F)
                            this.foundList(1,i) = coord(1);
                            this.foundList(2,i) = coord(2);
                            this.foundList(3,i) = F;
                            this.chromosomes(1,:) = encode(coord);
                            this.decodedChrom(:,1) = coord;
                        end
                    end
                end
                if (exist == false)
                    this.foundList(1,this.foundPointer) = coord(1);
                    this.foundList(2,this.foundPointer) = coord(2);
                    this.foundList(3,this.foundPointer) = F;
                    this.foundPointer = this.foundPointer+1;
                    ran = ceil((elitePop-1)*rand(1))+1;
                    ran = 1;
                    this.chromosomes(ran,:) = encode(coord);
                    this.decodedChrom(:,ran) = coord;
                    timestamp = time;
                end
            end
        end
        
        function[found]=getFoundList(this)
            found=this.foundList;
        end
        
        function[neighbours]=findNeighbours(this,rovers)
            global radioRange;
            pointer = 1;
            neighbours = 0;
            for r=1:size(rovers,2)
                distanceArray(r) = ((this.currentX-rovers(r).currentX)^2 + (this.currentY-rovers(r).currentY)^2)^(1/2);
                if (distanceArray(r)<radioRange && distanceArray(r)~=0)
                    neighbours(pointer) = r;
                    pointer = pointer+1;
                end    
            end
        end
        
        function[]=exchangeElite(this,rovers,neighbours)
            global elitePop
            if (neighbours~=0)
                for n=1:size(neighbours,2)
                     for e=1:elitePop
                         roverF(e)= getStrength(rovers(neighbours(n)).decodedChrom(1,e),rovers(neighbours(n)).decodedChrom(2,e));
                         thisF(e)= getStrength(this.decodedChrom(1,e),this.decodedChrom(2,e));
                     end
                     for e=1:elitePop
                         if(roverF(e)<thisF(e))
                            this.decodedChrom(:,e) = rovers(neighbours(n)).decodedChrom(:,e);
                            this.chromosomes(e,:) = rovers(neighbours(n)).chromosomes(e,:);
                         end
                         if(roverF(e)>thisF(e))
                            rovers(neighbours(n)).decodedChrom(:,e) = this.decodedChrom(:,e);
                            rovers(neighbours(n)).chromosomes(e,:) = this.chromosomes(e,:);
                         end
                     end
                end
            end
        end      
        
    end
end