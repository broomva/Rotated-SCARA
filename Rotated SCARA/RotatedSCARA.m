clc, clear, close all, warning off

% Rotated Scara Bot
% RRRT

% Spiral
r = 1; f=0.15; %r(.1-4) f(0.005-0.15)

t = linspace(0,10*pi,85);
x = r*sin(t)-6;
y = r*cos(t)-2;
z = f*t+3;


% Robot Parameters
a1 = 6; j1 = 2;
a2 = 2;
a3 = 2;
a4 = 5;

theta3 = 90;
rota4 = 1;
ang4 = 0;

% theta1 = zeros(size(t,2),1);
% theta2 = zeros(size(t,2),1);
% transl = zeros(size(t,2),1);
% theta4 = zeros(size(t,2),1);

% %---------------------- Inverse Kinematics --------------------------------
for h = 1:size(t,2)
    xf = x(h);
    yf = y(h);
    zf = z(h);
    flag = 0;
    
    % Restrictions
    if abs(xf+yf)<1
    disp('Non reachable')
    flag = 1;
    end

    d4 = 8-zf;
    if d4>a4 || d4<0
    disp('Non reachable')
    flag = 1;
    end
    
    M = (xf^2+yf^2-(a1)^2-(a4)^2)/(2*a1*a4);
    if (1-M^2)<0
    disp('Non reachable')
    flag = 1;
    end
    
    ang2 = atan2d((sqrt(1-M^2)),(M));
    ang1 = atan2d(yf,xf)-atan2d((a4*sind(ang2)),(a1+(a4*cosd(ang2))));
    ang4 = ang4+1;
    
    theta1(h) = ang1;
    theta2(h) = ang2;
    transl(h) = d4;
    theta4(h) = ang4;
end

%flag = 0;

% Unscrewed rotations
if theta3 == 90
    gain = 1; 
else
    gain = -1;
end

if rota4 == 0
    theta4 = zeros(1,size(theta4,2));
end


if flag ~= 1 
    %-------------------------- Links Definition --------------------------
    
    % Link 1
    L1 = NewLink([0 0 0], 1, a1, 300);
    xL1 = L1(1,:);  yL1 = L1(2,:);  zL1 = L1(3,:);  tL1 = L1(4,:);
    
    %--------------------------- Joints Definition ------------------------
    
    % Joint 1
    J1 = NewJoint([0 -1 6], 1, j1, 100);
    xJ1 = J1(1,:);  yJ1 = J1(2,:);  zJ1 = J1(3,:);  tJ1 = J1(4,:);
    
    % Joint 2
    L2 = NewLink([0 0 0], 1, a2, 300);
    xL2 = L2(1,:);  yL2 = L2(2,:);  zL2 = L2(3,:);  tL2 = L2(4,:);
    
    % Joint 3
    L3 = NewLink([0 0 0], 1, a3, 300);
    xL3 = L3(1,:);  yL3 = L3(2,:);  zL3 = L3(3,:);  tL3 = L3(4,:);
    
    % Joint 4
    L4 = NewLink([0 0 0],0.51,a4,300);
    xL4 = L4(1,:);  yL4 = L4(2,:);  zL4 = L4(3,:);  tL4 = L4(4,:);
    
    
    
    % --------------------------------- Tapas -----------------------------
    Tp1 = NewTapa([0 -1 6], 1, 0:.2:10*pi);
    xTp1 = Tp1(1,:); yTp1 = Tp1(2,:); zTp1 = Tp1(3,:); tTp1 = Tp1(4,:);
    Tpp1 = NewTapa([0 1 6], 1, 0:.2:10*pi);
    xTpp1 = Tpp1(1,:); yTpp1 = Tpp1(2,:); zTpp1 = Tpp1(3,:); tTpp1 = Tp1(4,:);
    Tp2 = NewTapa2([0 0 -.1], 1, 0:.2:10*pi);
    xTp2 = Tp2(1,:); yTp2 = Tp2(2,:); zTp2 = Tp2(3,:); tTp2 = Tp2(4,:);
    Tpp2 = NewTapa2([0 0 2.1], 1, 0:.2:10*pi);
    xTpp2 = Tpp2(1,:); yTpp2 = Tpp2(2,:); zTpp2 = Tpp2(3,:); tTpp2 = Tpp2(4,:);
    Tp3 = NewTapa2([0 0 -.1], 1, 0:.2:10*pi);
    xTp3 = Tp3(1,:); yTp3 = Tp3(2,:); zTp3 = Tp3(3,:); tTp3 = Tp3(4,:);
    Tpp3 = NewTapa2([0 0 2.1], 1, 0:.2:10*pi);
    xTpp3 = Tpp3(1,:); yTpp3 = Tpp3(2,:); zTpp3 = Tpp3(3,:); tTpp3 = Tpp3(4,:);
    
    
    % Base
    tb = 0:1/10000:1;
    xB = 5*sin(500*tb); 
    yB = 5*cos(900*tb); 
    zB = -1+tb;
    
    % Tapa Base
    tt1=0:.2:10*pi;
    xt1=1.5*cos(tt1); yt1=1.5*sin(tt1);zt1=zeros(size(tt1));
    xt2=1.2*cos(tt1); yt2=1.2*sin(tt1);zt2=5+zeros(size(tt1));
    
%     spiral = zeros(size(theta1,2),1);
    
    for j = 1:size(theta1,2)
        %---------------------------- Direct Kinematics -------------------
        R4 = MatrizRotacion(theta4(j),3);
        Tl4 = [0 0 0]';
        R0 = MatrizRotacion(theta3,1); %(1->x,2->y,3->z)
        Tl0 = [0 gain*4.5 6.5]';
        R1 = Denavit(theta1(j),4,6,0);
        R2 = Denavit(theta2(j),0,5,0);
        R3 = Denavit(0,-transl(j),0,0);
        
        % Homogeneous Transformation Matrices
        A0 = MTH(R4,Tl4);
        A1 = A0 * MTH(R0,Tl0);
        A2 = A1 * R1;
        A3 = A2 * R2;
        A4 = A3 * R3;
        
        % Transformations
        % Link 1
        TL1 = SinglePointTransformation(xL1,yL1,zL1,tL1,(A0));
        xTL1 = TL1(1,:);    yTL1 = TL1(2,:);    zTL1 = TL1(3,:);
        % Link 2
        p11=A2*[0 0 1 1]';
        p12=A2*[-6 0 1 1]';
        p13=A2*[-6 0 .2 1]';
        p14=A2*[0 0 .2 1]';
        v1 = [p11(1:3)';p12(1:3)';p13(1:3)';p14(1:3)'];
        f1 = [1 2 3 4];
        %Link 3
        p21=A3*[0 0 1.2 1]';
        p22=A3*[-6 0 1.2 1]';
        p23=A3*[-6 0 .2 1]';
        p24=A3*[0 0 .2 1]';
        v2 = [p21(1:3)';p22(1:3)';p23(1:3)';p24(1:3)'];
        f2 = [1 2 3 4];
        
        % Joints
        TJ1 = SinglePointTransformation(xJ1,yJ1,zJ1,tJ1,(A0));
        xTJ1 = TJ1(1,:);    yTJ1 = TJ1(2,:);    zTJ1 = TJ1(3,:);
        TL2 = SinglePointTransformation(xL2,yL2,zL2,tL2,(A2));
        xTL2 = TL2(1,:);    yTL2 = TL2(2,:);    zTL2 = TL2(3,:);
        TL3 = SinglePointTransformation(xL3,yL3,zL3,tL3,(A3));
        xTL3 = TL3(1,:);    yTL3 = TL3(2,:);    zTL3 = TL3(3,:);
        TL4 = SinglePointTransformation(xL4,yL4,zL4,tL4,(A4));
        xTL4 = TL4(1,:);    yTL4 = TL4(2,:);    zTL4 = TL4(3,:);
        
        % Tapas
        TTp1 = SinglePointTransformation(xTp1,yTp1,zTp1,tTp1,(A0));
        xTTp1 = TTp1(1,:); yTTp1 = TTp1(2,:); zTTp1 = TTp1(3,:);
        TTpp1 = SinglePointTransformation(xTpp1,yTpp1,zTpp1,tTpp1,(A0));
        xTTpp1 = TTpp1(1,:); yTTpp1 = TTpp1(2,:); zTTpp1 = TTpp1(3,:);
        TTp2 = SinglePointTransformation(xTp2,yTp2,zTp2,tTp2,(A2));
        xTTp2 = TTp2(1,:); yTTp2 = TTp2(2,:); zTTp2 = TTp2(3,:);
        TTpp2 = SinglePointTransformation(xTpp2,yTpp2,zTpp2,tTpp2,(A2));
        xTTpp2 = TTpp2(1,:); yTTpp2 = TTpp2(2,:); zTTpp2 = TTpp2(3,:);
        TTp3 = SinglePointTransformation(xTp3,yTp3,zTp3,tTp3,(A3));
        xTTp3 = TTp3(1,:); yTTp3 = TTp3(2,:); zTTp3 = TTp3(3,:);
        TTpp3 = SinglePointTransformation(xTpp3,yTpp3,zTpp3,tTpp3,(A3));
        xTTpp3 = TTpp3(1,:); yTTpp3 = TTpp3(2,:); zTTpp3 = TTpp3(3,:);
        
    
        % Plots
        axis([-15 15 -15 15 -1 15]);
        axis square, grid on, hold on
%         view(-theta4(j),60)
        
        rectangle('Position',[-5,-5,10,10],'Curvature',[0 0],'FaceColor',[0 0.21 0.22],'EdgeColor','k','LineWidth',1)  
%       rectangle('Position',[-15,-15,30,30],'Curvature',[0 0],'FaceColor',[0 0.21 0.22],'EdgeColor','k','LineWidth',1)

        plot3(xB,yB,zB,'Color',[0 0.21 0.22]);
        plot3(xTL1,yTL1,zTL1,'b');
        plot3(xTL2,yTL2,zTL2,'b');
        plot3(xTL3,yTL3,zTL3,'b');
        plot3(xTL4,yTL4,zTL4,'g');

        plot3(xTJ1,yTJ1,zTJ1,'b');
  
%         EndEffector(:,j) = A4*[0,0,0,1]';
%         plot3(EndEffector(1,:),EndEffector(2,:),EndEffector(3,:),'*')

        % Patchs
        % Base - Link 1
        patch(xt1,yt1,zt1,'b');
        patch(xt2,yt2,zt2,'b');
        
        % Links
        patch('Faces',f1,'Vertices',v1,'FaceColor','b','EdgeColor','b','LineWidth',3)
        patch('Faces',f2,'Vertices',v2,'FaceColor','b','EdgeColor','b','LineWidth',3)

       % Joints
        patch(xTTp1,yTTp1,zTTp1,'b');
        patch(xTTpp1,yTTpp1,zTTpp1,'b');
        patch(xTTp2,yTTp2,zTTp2,'b');
        patch(xTTpp2,yTTpp2,zTTpp2,'b');
        patch(xTTp3,yTTp3,zTTp3,'k');
        patch(xTTpp3,yTTpp3,zTTpp3,'k');
       
        pause(0.01);
        clf
        
        spiral(:,j) = A4*[0,0,0,1]';

        plot3(spiral(1,:),spiral(2,:),spiral(3,:),'k')
    end
    
axis([-15 15 -15 15 -1 15]);
axis square,grid on,hold on
% view(-theta4(j),60)
    
% Plots
rectangle('Position',[-5,-5,10,10],'Curvature',[0 0],'FaceColor',[0 0.21 0.22],'EdgeColor','k','LineWidth',1)

% rectangle('Position',[-10,-10,15,15],'Curvature',[0 0],'FaceColor',[0 0.21 0.22],'EdgeColor','k','LineWidth',1)

plotB = plot3(xB,yB,zB,'Color',[0 0.21 0.22]);
plotL1 = plot3(xTL1,yTL1,zTL1,'b');
plotL2 = plot3(xTL2,yTL2,zTL2,'b');
plotL3 = plot3(xTL3,yTL3,zTL3,'b');
plotL4 = plot3(xTL4,yTL4,zTL4,'g');

plotJ1 = plot3(xTJ1,yTJ1,zTJ1,'b');


% Patchs
% Base - Link 1
patch(xt1,yt1,zt1,'b');
patch(xt2,yt2,zt2,'b');

% Links
patch('Faces',f1,'Vertices',v1,'FaceColor','b','EdgeColor','b','LineWidth',3)
patch('Faces',f2,'Vertices',v2,'FaceColor','b','EdgeColor','b','LineWidth',3)


% Joints
patch(xTTp1,yTTp1,zTTp1,'b');
patch(xTTpp1,yTTpp1,zTTpp1,'b');
patch(xTTp2,yTTp2,zTTp2,'b');
patch(xTTpp2,yTTpp2,zTTpp2,'b');
patch(xTTp3,yTTp3,zTTp3,'k');
patch(xTTpp3,yTTpp3,zTTpp3,'k');
    
end
