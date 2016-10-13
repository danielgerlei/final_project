% Mutation Function
% inputs:
%        Mutation rate
%        Number of Children
%        Length of chromosomes
%        Matrix of unmutated children
% output: 
%        Matrix of mutated children
function[kinder]=mutate(murate,chinum,chrlen,kinder)

ptemp=1;                                    % initial starting point
MutPart=100/murate;                         % mutation partition
ChangeNum=round((chinum*chrlen)/MutPart);   % number of changes
Range=floor((chinum*chrlen)/ChangeNum);     % range of possible points
total=ChangeNum*Range;
for kin=1:ChangeNum,                        % loop to mutate number of points
	pt=ptemp+round(ChangeNum*rand(1));		% point selection
	if pt>=total
       pt=ceil(total*rand(1));
	end
	row=ceil(pt/chrlen);                    % row position of point
	col=rem(pt,chrlen);                     % column position of point
	if col==0                               % if column position comes out as 0, it is reset to 10...
		col=chrlen;                         % ... to avoid errors
	end
	kinder(row,col)=round(9*rand(1));       % child chromosome is randomly changed
	ptemp=Range*kin;                        % new start point is selected to encourage good distribution of mutated points
end
