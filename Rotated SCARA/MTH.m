function HMatrix = MTH(R,Tl)
HMatrix = [R(1,:) Tl(1); R(2,:) Tl(2); R(3,:) Tl(3); 0 0 0 1];