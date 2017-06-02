%% 机器人参数的定义
HipOffsetY=50;
HipOffsetZ=85;
ThighLength=100;
TibiaLength=102.9;
FootHeight=45.19;
NeckOffsetZ=126.50;
ShoulderOffsetY=98;
UpperArmLength=105;
LowerArmLength=55.95;
HandOffsetX=57.75;
%% coordinate system
% a and b for leg
%au and bu for up body
a=[ 0 0 1 0 0 0 1 0 1 0 0 0 1;
    0 sqrt(2)/2 0 1 1 1 0 sqrt(2)/2 0 1 1 1 0;
    1 sqrt(2)/2 0 0 0 0 0 -sqrt(2)/2 0 0 0 0 0];
b=[0 0 0 0 0 0 0 0 0 0 0 0 0;
    0 -HipOffsetY 0 0 0 0 0 HipOffsetY 0 0 0 0 0;
    0 0 0 0 -ThighLength -TibiaLength 0 0 0 0 -ThighLength -TibiaLength 0];

%%
T1=@(x)([[eye(3)+[0 -a(3,1) a(2,1);a(3,1) 0 -a(1,1);-a(2,1) a(1,1) 0]*sin(x)+[0 -a(3,1) a(2,1);a(3,1) 0 -a(1,1);-a(2,1) a(1,1) 0]*[0 -a(3,1) a(2,1);a(3,1) 0 -a(1,1);-a(2,1) a(1,1) 0]*(1-cos(x))],b(:,1);0 0 0 1]);
T2=@(x)([[eye(3)+[0 -a(3,2) a(2,2);a(3,2) 0 -a(1,2);-a(2,2) a(1,2) 0]*sin(x)+[0 -a(3,2) a(2,2);a(3,2) 0 -a(1,2);-a(2,2) a(1,2) 0]*[0 -a(3,2) a(2,2);a(3,2) 0 -a(1,2);-a(2,2) a(1,2) 0]*(1-cos(x))],b(:,2);0 0 0 1]);
T3=@(x)([[eye(3)+[0 -a(3,3) a(2,3);a(3,3) 0 -a(1,3);-a(2,3) a(1,3) 0]*sin(x)+[0 -a(3,3) a(2,3);a(3,3) 0 -a(1,3);-a(2,3) a(1,3) 0]*[0 -a(3,3) a(2,3);a(3,3) 0 -a(1,3);-a(2,3) a(1,3) 0]*(1-cos(x))],b(:,3);0 0 0 1]);
T4=@(x)([[eye(3)+[0 -a(3,4) a(2,4);a(3,4) 0 -a(1,4);-a(2,4) a(1,4) 0]*sin(x)+[0 -a(3,4) a(2,4);a(3,4) 0 -a(1,4);-a(2,4) a(1,4) 0]*[0 -a(3,4) a(2,4);a(3,4) 0 -a(1,4);-a(2,4) a(1,4) 0]*(1-cos(x))],b(:,4);0 0 0 1]);
T5=@(x)([[eye(3)+[0 -a(3,5) a(2,5);a(3,5) 0 -a(1,5);-a(2,5) a(1,5) 0]*sin(x)+[0 -a(3,5) a(2,5);a(3,5) 0 -a(1,5);-a(2,5) a(1,5) 0]*[0 -a(3,5) a(2,5);a(3,5) 0 -a(1,5);-a(2,5) a(1,5) 0]*(1-cos(x))],b(:,5);0 0 0 1]);
T6=@(x)([[eye(3)+[0 -a(3,6) a(2,6);a(3,6) 0 -a(1,6);-a(2,6) a(1,6) 0]*sin(x)+[0 -a(3,6) a(2,6);a(3,6) 0 -a(1,6);-a(2,6) a(1,6) 0]*[0 -a(3,6) a(2,6);a(3,6) 0 -a(1,6);-a(2,6) a(1,6) 0]*(1-cos(x))],b(:,6);0 0 0 1]);
T7=@(x)([[eye(3)+[0 -a(3,7) a(2,7);a(3,7) 0 -a(1,7);-a(2,7) a(1,7) 0]*sin(x)+[0 -a(3,7) a(2,7);a(3,7) 0 -a(1,7);-a(2,7) a(1,7) 0]*[0 -a(3,7) a(2,7);a(3,7) 0 -a(1,7);-a(2,7) a(1,7) 0]*(1-cos(x))],b(:,7);0 0 0 1]);
T8=@(x)([[eye(3)+[0 -a(3,8) a(2,8);a(3,8) 0 -a(1,8);-a(2,8) a(1,8) 0]*sin(x)+[0 -a(3,8) a(2,8);a(3,8) 0 -a(1,8);-a(2,8) a(1,8) 0]*[0 -a(3,8) a(2,8);a(3,8) 0 -a(1,8);-a(2,8) a(1,8) 0]*(1-cos(x))],b(:,8);0 0 0 1]);
T9=@(x)([[eye(3)+[0 -a(3,9) a(2,9);a(3,9) 0 -a(1,9);-a(2,9) a(1,9) 0]*sin(x)+[0 -a(3,9) a(2,9);a(3,9) 0 -a(1,9);-a(2,9) a(1,9) 0]*[0 -a(3,9) a(2,9);a(3,9) 0 -a(1,9);-a(2,9) a(1,9) 0]*(1-cos(x))],b(:,9);0 0 0 1]);
T10=@(x)([[eye(3)+[0 -a(3,10) a(2,10);a(3,10) 0 -a(1,10);-a(2,10) a(1,10) 0]*sin(x)+[0 -a(3,10) a(2,10);a(3,10) 0 -a(1,10);-a(2,10) a(1,10) 0]*[0 -a(3,10) a(2,10);a(3,10) 0 -a(1,10);-a(2,10) a(1,10) 0]*(1-cos(x))],b(:,10);0 0 0 1]);
T11=@(x)([[eye(3)+[0 -a(3,11) a(2,11);a(3,11) 0 -a(1,11);-a(2,11) a(1,11) 0]*sin(x)+[0 -a(3,11) a(2,11);a(3,11) 0 -a(1,11);-a(2,11) a(1,11) 0]*[0 -a(3,11) a(2,11);a(3,11) 0 -a(1,11);-a(2,11) a(1,11) 0]*(1-cos(x))],b(:,11);0 0 0 1]);
T12=@(x)([[eye(3)+[0 -a(3,12) a(2,12);a(3,12) 0 -a(1,12);-a(2,12) a(1,12) 0]*sin(x)+[0 -a(3,12) a(2,12);a(3,12) 0 -a(1,12);-a(2,12) a(1,12) 0]*[0 -a(3,12) a(2,12);a(3,12) 0 -a(1,12);-a(2,12) a(1,12) 0]*(1-cos(x))],b(:,12);0 0 0 1]);
T13=@(x)([[eye(3)+[0 -a(3,13) a(2,13);a(3,13) 0 -a(1,13);-a(2,13) a(1,13) 0]*sin(x)+[0 -a(3,13) a(2,13);a(3,13) 0 -a(1,13);-a(2,13) a(1,13) 0]*[0 -a(3,13) a(2,13);a(3,13) 0 -a(1,13);-a(2,13) a(1,13) 0]*(1-cos(x))],b(:,13);0 0 0 1]);

%%
s=0.08;
z=0.23;
yy=25;
y=60;
g=9.8;
vf=0.5*(2*s/(exp(pi)-1)+s)*sqrt(g/z);
T=pi/sqrt(g/z);
t=0:T/9:T;
x=1000*((vf/2.*sqrt(z/g)-s/4).*exp(sqrt(g/z).*t)-(vf/2.*sqrt(z/g)+s/4).*exp(-sqrt(g/z).*t));
L_standx=-x;
L_standy=50-y*sin(sqrt(g/z)*t);
L_standz=ones(1,10)*-z*1000;

x0=1000*((vf/2.*sqrt(z/g)-s/4).*1-(vf/2.*sqrt(z/g)+s/4).*1);
R_movex=x;
R_movey=-120*ones(size(L_standy))+L_standy(5);
R_movez=-z*ones(1,10)*1000+yy*sin(sqrt(g/z)*t);

R_standx=-x;
R_standy=-50+y*sin(sqrt(g/z)*t);
R_standz=-z*ones(1,10)*1000;

L_movex=x;
L_movey=120*ones(size(L_standy))+R_standy(5);
L_movez=-z*1000*ones(1,10)+yy*sin(sqrt(g/z)*t);

%start
LR_x0=0;
LR_y0=0;
LR_z0=-z*1000;
R_startx1=-[0 0 0 0 0 x(6) x(7) x(8) x(9) x(10)];
%转移重心
R_startx2=-[0 0 0 0 0 0 0 0 0 0];      
%转移腿
R_starty1=-50+y*sin(sqrt(g/z)*t);      
%转移重心
R_starty2=[R_starty1(5) R_starty1(5) R_starty1(5) R_starty1(5) R_starty1(5) R_starty1(5) R_starty1(5) R_starty1(5) R_starty1(5) R_starty1(5)];  %转移腿（10）
R_startz=ones(1,10)*-z*1000;
L_startx=[x(6) x(6) x(7) x(7) x(8) x(8) x(9) x(9) x(10) x(10)];
L_starty1=100*ones(size(R_starty1))+R_starty1;
L_starty2=100*ones(size(R_starty2))+R_starty2;
L_startz1=ones(1,10)*-z*1000;
L_startz2=-z*ones(1,10)*1000+yy*sin(sqrt(g/z)*t);
positionstart=[LR_x0,R_startx1(1:5),R_startx1(6:10)-2*(x0)*ones(1,5),L_startx;
    LR_y0,L_starty1,L_starty2;
    LR_z0,L_startz1,L_startz2;
    ones(1,21);
    LR_x0,R_startx1,R_startx2;
    LR_y0,R_starty1,R_starty2;
    LR_z0,R_startz,R_startz;
    ones(1,21)];
    position=[L_standx,ones(1,10)*L_standx(10),R_standx(1:5)-2*(-x0)*ones(1,5),R_standx(6:10)-2*(x0)*ones(1,5),L_movex;
    L_standy,ones(1,10)*L_standy(10),R_standy+120*ones(1,10),L_movey;
    L_standz,ones(1,10)*L_standz(10),ones(1,10)*L_standz(10),L_movez;
    ones(1,40);
    L_standx(1:5)-2*(-x0)*ones(1,5),L_standx(6:10)-2*(x0)*ones(1,5),R_movex,R_standx,R_standx(10)*ones(1,10);
    L_standy-120*ones(1,10),R_movey,R_standy,R_standy(10)*ones(1,10);
    -z*10^3*ones(1,10),R_movez,R_standz,R_standz(10)*ones(1,10);
    ones(1,40)];
positionleft=position(1:4,:);
positionright=position(5:8,:);
leftstart=positionstart(1:4,:);
rightstart=positionstart(5:8,:);

%% solve function
angleleft=zeros(6,length(position(1,:)));
angleright=zeros(6,length(position(1,:)));
x0=[-0.1 -0.1 0.1 -0.01];
y0=[-0.1 -0.1 0.1 -0.01];
for i=1:40
myfun=@(x)(T1(0)*T2(0)*T3(x(1))*T4(x(2))*T5(x(3))*T6(-x(2)-x(3))*T7(x(4))*[0;0;-45.19;1]-positionright(:,i));
x=fsolve(@(x) myfun(x),x0);
angleright(:,i)=[0;x(1);x(2);x(3);(-x(2)-x(3));-x(1)*1.2];

myfun1=@(y)(T1(0)*T8(0)*T9(y(1))*T10(y(2))*T11(y(3))*T12(-y(2)-y(3))*T13(y(4))*[0;0;-45.19;1]-positionleft(:,i));
y=fsolve(@(y) myfun1(y),y0);
angleleft(:,i)=[0;y(1);y(2);y(3);-y(2)-y(3);-y(1)*1.2];
end

angleleft=[angleleft(:,1:5),[angleleft(:,5),angleleft(:,5),angleleft(:,5),angleleft(:,5),angleleft(:,5),angleleft(:,5),angleleft(:,5),angleleft(:,5),angleleft(:,5),angleleft(:,5)],angleleft(:,6:10),angleleft(:,21:25),angleleft(:,31:40),angleleft(:,26:30)];
angleright=[angleright(:,1:5),angleright(:,11:20),angleright(:,6:10),angleright(:,21:25),[angleright(:,25),angleright(:,25),angleright(:,25),angleright(:,25),angleright(:,25),angleright(:,25),angleright(:,25),angleright(:,25),angleright(:,25),angleright(:,25)],angleright(:,26:30)];
angleleftstart=zeros(6,11);
anglerightstart=zeros(6,11);
x0=[0 -0.5 0.5 0];
y0=[0 -0.5 0.5 0];
for i=1:21
myfun=@(x)(T1(0)*T2(0)*T3(x(1))*T4(x(2))*T5(x(3))*T6(-x(2)-x(3))*T7(x(4))*[0;0;-45.19;1]-rightstart(:,i));
x=fsolve(@(x) myfun(x),x0);
anglerightstart(:,i)=[0;x(1);x(2);x(3);(-x(2)-x(3));-x(1)*1.2];
myfun1=@(y)(T1(0)*T8(0)*T9(y(1))*T10(y(2))*T11(y(3))*T12(-y(2)-y(3))*T13(y(4))*[0;0;-45.19;1]-leftstart(:,i));
y=fsolve(@(y) myfun1(y),y0);
angleleftstart(:,i)=[0;y(1);y(2);y(3);-y(2)-y(3);-y(1)*1.2];
end
angleleftstart=[angleleftstart(:,1:6),angleleftstart(:,12:21),angleleftstart(:,7:11)];
anglerightstart=[anglerightstart(:,1:6),anglerightstart(:,12:21),anglerightstart(:,7:11)];
left=[angleleftstart,angleleft];
right=[anglerightstart,angleright];

for j=1:10
    for i=2:21
        angle_leg=[right(:,i); left(:,i)]; 
        angle_shoulder=zeros(1,10);
        angle2posnew(angle_leg,angle_shoulder);
    end
end