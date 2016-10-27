function[]=writeTime(time)
    m = floor(time/60);
    s = round(time-m*60);
    t = [m,s];
    disp('time passed (min sec)')
    disp(t)
    