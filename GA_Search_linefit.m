% Genetic Algorithms in Matlab (minimising fitness)

clear

% Initial Simulation Data

i=1;                            % counter
targetX=3;                      % gradient of target line
targetY=5;                      % displacement of target line
figure(1)                       
clf
axis([0 10 0 10])  
plot(targetX,targetY,'ro'),hold on          % plot target point in red
pause(1e-10)

for x = -10:0.2:10,				% desired response
   ydesout(i)=targetX*x+targetY;
   xdesout(i)=x;
   i=i+1;
end

% Initial Data

genNum=60;                      % number of generations
popSize=30;                     % size of population
parNum=2;                       % number of parents (number of dimensions)
chrLen=10;                      % length of chromosomes
childNum=round(0.8*popSize);    % number of children
popSize2=popSize-childNum;      % number of elite agents
num=(popSize2+1):popSize;       % vector to point at the rest of the population
mutRate=5;                      % mutation rate
bot = 1;                        % floor to start loops from


% Creating Initial Chromosomes 
[chrom,coord]=createFirstGen(popSize,chrLen,parNum);

plot(coord(1,:),coord(2,:),'mo')                        % plot first generation in magenta
pause(1e-50)

[F,y]=rankGen1(coord,popSize,ydesout);                  % evaluate the first generation

[chrom,coord,F]=sort_swap(chrom,coord,F,popSize);		% sort outcomes according to F score for reproduction (elitism)

% create and evaluate following generations of chromosomes
for gen=bot:genNum,
	kinder=crossover2p(popSize,childNum,chrom,chrLen);              % 2 point crossover to create children
	kinder=mutate(mutRate,childNum,chrLen,kinder);                  % mutation to ensure genetic diversity
	kidCoord=decode(kinder,childNum,parNum);                        % decoding children
    [coord,chrom]=assembleGen(coord,chrom,num,kidCoord,kinder);     % assemble new generation, keeping the elite
    F=rank(coord,popSize2,childNum,ydesout,y,F);                    % evaluate new generation

    % plot results
    axis([0 10 0 10]) 
    % scale the clour of the plot to make distinguishing between
    % generations plausible
    plot(coord(1,:),coord(2,:),'Marker','o','MarkerEdgeColor',[0.5*gen/genNum 1*gen/genNum 0.3*gen/genNum],'LineStyle','none'),xlabel('c'), ylabel('m'),hold on
    pause(1e-40)
    plot(targetX,targetY,'ro')
 %  plot(q2(1,:),q2(2,:),'wo')
    [chrom,coord,F]=sort_swap(chrom,coord,F,popSize);	% sort new generation
   
end
m=coord(1,1)		% display best results
c=coord(2,1)
i=1;

% create resulting line
for x = -10:0.2:10,
  	y(i)=m*x+c;
  	i=i+1;
end
x = -10:0.2:10;

% Plot results

figure(2)
clf

plot(x,y,'b'),xlabel('x'),ylabel('y'),hold on
plot(x,ydesout,'r--')
