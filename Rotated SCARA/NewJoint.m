function Points = NewJoint(pos, diam, lenght, scale)
xi = pos(1);
yi = pos(2);
zi = pos(3);
t = 0:1/100:lenght;
xEi = diam*sin(scale*t)+xi;
zEi = diam*cos(scale*t)+zi;
yEi = t+yi;
Points = [xEi; yEi; zEi; t];

