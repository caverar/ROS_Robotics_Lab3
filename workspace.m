%% Lab 2 - Matlab and ROS with PhantomX
clear
clc

l = [14.5, 10.63, 10.65, 8.97]; % Length of links
L(1) = Link('revolute','alpha',pi/2,'a',0,   'd',l(1),'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(2) = Link('revolute','alpha',0,   'a',l(2),'d',0,   'offset',pi/2,'qlim',[-3*pi/4 3*pi/4]);
L(3) = Link('revolute','alpha',0,   'a',l(3),'d',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
L(4) = Link('revolute','alpha',0,   'a',0,   'd',0,   'offset',0,   'qlim',[-3*pi/4 3*pi/4]);
PhantomX = SerialLink(L,'name','Px');

%Tool orientation following NAO Convention
PhantomX.tool = [0 0 1 l(4); -1 0 0 0; 0 -1 0 0; 0 0 0 1];

%% 3d workspace
rng('default')
samples = 1000000;
q1 = ((3*pi/2)*rand(samples,1))-((3*pi/4)*(ones(samples,1)));
q2 = ((3*pi/2)*rand(samples,1))-((3*pi/4)*(ones(samples,1)));
q3 = ((3*pi/2)*rand(samples,1))-((3*pi/4)*(ones(samples,1)));
q4 = ((3*pi/2)*rand(samples,1))-((3*pi/4)*(ones(samples,1)));

x_pos = zeros(samples,1);
y_pos = zeros(samples,1);
z_pos = zeros(samples,1);
disp(x_pos(1))

for i = 1:1:samples
    MTH = PhantomX.fkine([q1(i) q2(i) q3(i) q4(i)]);
    x_pos(i) = MTH(1,4);
    y_pos(i) = MTH(2,4);
    z_pos(i) = MTH(3,4);

end

%plot3(x_pos,y_pos,z_pos);
bound=boundary(x_pos,y_pos,z_pos);
trisurf(bound,x_pos,y_pos,z_pos,'Facecolor','red','FaceAlpha',0.1)



%% 2d workspace

rng('default')
2dsamples = 1000000;

q2 = ((3*pi/2)*rand(2dsamples,1))-((3*pi/4)*(ones(2dsamples,1)));
q3 = ((3*pi/2)*rand(2dsamples,1))-((3*pi/4)*(ones(2dsamples,1)));
q4 = ((3*pi/2)*rand(2dsamples,1))-((3*pi/4)*(ones(2dsamples,1)));

x_pos = zeros(2dsamples,1);
y_pos = zeros(2dsamples,1);

disp(x_pos(1))

for i = 1:1:2dsamples
    MTH = PhantomX.fkine([0 q2(i) q3(i) q4(i)]);
    x_pos(i) = MTH(1,4);
    y_pos(i) = MTH(2,4);
end

%plot3(x_pos,y_pos,z_pos);
bound=boundary(x_pos,y_pos);
plot(x_pos(bound),y_pos(bound))

