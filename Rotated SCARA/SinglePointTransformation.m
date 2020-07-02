function pointsT = SinglePointTransformation(x,y,z,vectsize,Ai)
%Ai = DHparameters(thetai,di,ai,alphai);
for i=1:size(vectsize,2)
        points = [x(i) y(i) z(i) 1];  % Data points at i of x y z are stored in a vector
        Tp = Ai*points'; % Points are operated with Transformation matrix
        xT(i) = Tp(1); % Transformated points are stored in a vector
        yT(i) = Tp(2); % for every x y z rotated-translated
        zT(i) = Tp(3);
end

pointsT = [xT; yT; zT];
