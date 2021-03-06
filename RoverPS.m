
classdef RoverPS < handle
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
        personalBest
        bestPosition
        velVector
    end
    
    methods
        function[this]=RoverPS(x,y)
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
            this.personalBest = Fmax;
            this.bestPosition  = [x,y];
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
            Ka = 1;             % attraction constant
            Kr = -15000;        % repulsion constant
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
                [Xvelocity,Yvelocity]=limitSpeed(Xvelocity,Yvelocity,maxSpeed);
                % if the craft is stuck, generate random velocity
                [Xvelocity,Yvelocity,slow]=unStuck(Xvelocity,Yvelocity,stuck,slow,slowSet,Kran);
                X = Euler(X,Xvelocity,step);         % Euler forward integration to get position      
                Y = Euler(Y,Yvelocity,step);
                % store data 
                coord=[X,Y];
                this.updatePosition(X,Y);
                this.Xvel = Xvelocity;
                this.Yvel = Yvelocity;
                this.velVector = [this.Xvel,this.Yvel];
                XObstacle(NoObstacles+roverPointer) = this.currentX;
                YObstacle(NoObstacles+roverPointer) = this.currentY;
                this.savePath();
                strength=getStrength(X,Y);
                timestamp = this.saveTarg(coord,strength,time);
                this.arrived = false;
            else
                this.arrived = true;
            end
        end
        
        function[timestamp]=getStepPS(this,bestPosition,maxSpeed,step,roverPointer,time)
            global NoObstacles;                             
            global XObstacle;                       
            global YObstacle;
            global KrPersonal;
            global KrGlobal;
            Krep = 150;
            this.velVector = this.velVector+(KrPersonal*rand(1)).*(this.bestPosition-[this.currentX,this.currentY])+(KrGlobal*rand(1)).*(bestPosition-[this.currentX,this.currentY]);
            [frx,fry] = getRepulsive(this.currentX,this.currentY,Krep);
            vrx = frx*step;
            vry = fry*step;
            this.velVector = this.velVector+[vrx,vry];
            [this.velVector(1),this.velVector(2)]=limitSpeed(this.velVector(1),this.velVector(2),maxSpeed);
            X = this.currentX+this.velVector(1)*step;
            Y = this.currentY+this.velVector(2)*step;
            %store data
            coord=[X,Y];
            this.Xvel = this.velVector(1);
            this.Yvel = this.velVector(2);
            this.updatePosition(X,Y);
            XObstacle(NoObstacles+roverPointer) = this.currentX;
            YObstacle(NoObstacles+roverPointer) = this.currentY;
            this.savePath();
            strength=getStrength(X,Y);
            timestamp = this.saveTarg(coord,strength,time);
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
                        end
                    end
                end
                if (exist == false)
                    this.foundList(1,this.foundPointer) = coord(1);
                    this.foundList(2,this.foundPointer) = coord(2);
                    this.foundList(3,this.foundPointer) = F;
                    this.foundPointer = this.foundPointer+1;
                    timestamp = time;
                end
            end
        end
        
        function[found]=getFoundList(this)
            found=this.foundList;
        end
        
        function[]=updateBest(this,F)
            if(F <= this.personalBest) % <= makes it so that personal best is ignored until a point of significance is found
                this.personalBest = F;
                this.bestPosition  = [this.currentX,this.currentY];
            end
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
    end
end