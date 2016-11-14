function[]=getCoverage()
global RObstacle;
Xmin = 115;     % size of the image saved
Xmax = 794;
Ymin = 69;
Ymax = 606;
Image = imread('coverage.bmp');
BW = im2bw(Image,0.999999999999);
BW1=double(BW);
WhitePix = 0;
BlackPix = 0;
for j=Xmin:Xmax
    for i=Ymin:Ymax
        if BW1(i,j)==1
            WhitePix=WhitePix+1;
        else
            BlackPix=BlackPix+1;
        end
    end
end
BlackPix = BlackPix-2093; % substract pixels of the ruler
obstacles = sum((RObstacle.^2)*pi*3.395);
coverage = 100*(BlackPix/((Xmax*Ymax)-obstacles));
disp('percentage of area covered')
disp(coverage)