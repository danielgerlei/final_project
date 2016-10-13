% function to replace the non elite of the previous population with the
% newly created children

function[coord,chrom]=assembleGen(coord,chrom,num,kidCoord,kinder)
coord(:,num)=kidCoord;                              
chrom(num,:)=kinder;                                