%Sort and Swap function
% inputs:
%         Chromosomes
%         Decoded chromosomes
%         F function scores
%         population size
% outputs:
%         input matrices ranked by F values
function[chrom,q,F]=sort_swap(chrom,q,F,popu)

[Y,I]=sort(F);

for jind=1:popu,
	swap1(jind,:)=chrom(I(jind),:);  % reorder chromosome matrix according to F score
	swap2(:,jind)=q(:,I(jind));      % reorder decoded matrix according to F score
end
chrom=swap1;
q=swap2;
F=Y;