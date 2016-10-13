% Crossover function (ranking)
% inputs:
%        Population size
%        Number of children
%        Chromosomes
%        Length of Chromosomes
% output:
%        New generation of Chromosomes 
function[kinder]=crossover2p(popsize,childNum,chrom,chrlen)
popmag = popsize;
cnt=0;                                  % counter initialisation
chrlim1=chrlen-1;                       % chromosome limit 1
chrlim2=chrlen-2;                       % chromosome limit 2
	
rand('seed',sum(100*clock));			% randomisation seed initialisation 
% loop to crossover the population 
for jin=1:round(childNum/2),            % run for half the number of children, since each iteration creates two
	pt1=round(chrlim2*rand(1))+1;       % find first crossover point
	if pt1==chrlim1                     % if first point is one less than the chromosome length...
		pt2=chrlen;                     % ... the second point will be at the end of the chromosome
	else			
		pt2=round((chrlen-pt1)*rand(1))+pt1;	% if not then the second point is somewhere above the first
	end
	cnt=cnt+2;                          % increment counter by 2 (since 2 parents are handled at the one time
	cnt1=ceil(popmag*rand(1));			% select first parent chromosome
	cnt2=ceil(popmag*rand(1));			% select second parent chromosome
	while cnt1==cnt2,                   % check to make sure that the same parent isn't selected twice
		cnt2=ceil(popmag*rand(1));		% change thsecond parent if this is so
	end
	parent1(1,:)=chrom(cnt1,:);			% first parent selected 
	parent2(1,:)=chrom(cnt2,:);			% second parent selected
	child1=parent1;                     % first child is initially identical to first parent
	child2=parent2;                     % second child is initially identical to second parent
	kin=pt1:pt2;                        % range of section to be exchanged 
	child1(kin)=parent2(kin);			% part of second parent is inplanted into first child
	child2(kin)=parent1(kin);			% part of first parent is inplanted into second child
	kinder(cnt-1,:)=child1;				% new children are stored
	kinder(cnt,:)=child2;
end

