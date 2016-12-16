function[]=displayResults(rovers,roverNum,sensorFootprint)
global NoObstacles
global XObstacle
global YObstacle
global RObstacle
global targetRarray
global targetXarray
global targetYarray
global edgeLength
figure (1)
clf
 for r=1:roverNum
     [pathX,pathY]=rovers(r).getPathList();
     plot(pathX,pathY) 
     hold on
 end
 ang = 0:0.01:2*pi;
for n=1:NoObstacles
    xp=RObstacle(n)*cos(ang);
    yp=RObstacle(n)*sin(ang);
    plot(XObstacle(n)+xp,YObstacle(n)+yp,'k');
end
for n=1:size(targetXarray,2)
    xp=targetRarray(n)*cos(ang);
    yp=targetRarray(n)*sin(ang);
    plot(targetXarray(n)+xp,targetYarray(n)+yp,'r');
end
 plot(targetXarray,targetYarray,'rx')
 axis([0 edgeLength 0 edgeLength])
 figure (2)
 clf
for r=1:roverNum
     fl=rovers(r).getFoundList();
     plot(fl(1,:),fl(2,:),'Marker','o','MarkerEdgeColor',[0.5*r/roverNum 1*r/roverNum 0.3*r/roverNum],'LineStyle','none')
     hold on
end
plot(targetXarray,targetYarray,'ro')
axis([0 edgeLength 0 edgeLength])
 
figure (3)
clf
for r=1:roverNum
    [pathX,pathY]=rovers(r).getPathList();
    plot(pathX,pathY,'k','LineWidth',sensorFootprint*2) 
    hold on
end
axis([0 edgeLength 0 edgeLength])
saveas(gcf,'coverage.bmp','bmp') 
getCoverage();
  