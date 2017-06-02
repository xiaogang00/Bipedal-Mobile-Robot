%% 踢球的动作  
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

t=1:1:10;
L_downz=(ThighLength+TibiaLength+FootHeight-10)*ones(size(t))-5*t;
L_downx=zeros(size(t));
L_downy=zeros(size(t))+HipOffsetY-5*t;
R_downz=(ThighLength+TibiaLength+FootHeight-10)*ones(size(t))-5*t;
R_downx=zeros(size(t));
R_downy=-5*t-HipOffsetY;

L_standz=(ThighLength+TibiaLength+FootHeight-10)*ones(size(t))-50;
L_standx=zeros(size(t));
L_standy=zeros(size(t))+HipOffsetY*0;
R_movex=8*t;
R_movey=-50*ones(1,10)-HipOffsetY;
R_movez=(ThighLength+TibiaLength+FootHeight-10)*ones(size(t))-70;

L_upz=(ThighLength+TibiaLength+FootHeight-10)*ones(size(t))-50+5*t;
L_upx=zeros(size(t));
L_upy=zeros(size(t))+0*HipOffsetY+t/10*1*HipOffsetY;
R_upz=(ThighLength+TibiaLength+FootHeight-10)*ones(size(t))-60+6*t;
R_upx=80-8*t;
R_upy=-50-HipOffsetY+5*t;

positionleft=[L_downx L_standx L_upx; L_downy L_standy L_upy; -L_downz -L_standz -L_upz; ones(1,30)];
positionright=[R_downx R_movex R_upx; R_downy R_movey R_upy; -R_downz -R_movez -R_upz; ones(1,30)];

% solve function
angleleft=zeros(6,30);
angleright=zeros(6,30);
x0=[-0.2 -0.1 0];
y0=[-0.2 -0.1 0];
for i=1:30
myfun=@(x)(T1(0)*T2(0)*T3(x(1))*T4(x(2))*T5(x(3))*T6(-x(2)-x(3))*T7(-x(1))*[0;0;-45.19;1]-positionright(:,i));
options=optimoptions(@fsolve,'Algorithm','levenberg-marquardt','Display','Off');
x=fsolve(@(x) myfun(x),x0,options);
x0=x;
angleright(2:4,i)=x;
angleright(5,i)=-x(2)-x(3);
angleright(6,i)=-x(1);
myfun1=@(y)(T1(0)*T8(0)*T9(y(1))*T10(y(2))*T11(y(3))*T12(-y(2)-y(3))*T13(-y(1))*[0;0;-45.19;1]-positionleft(:,i));
y=fsolve(@(y) myfun1(y),y0,options);
y0=y;
angleleft(:,i)=[0;y(1:3)';-y(2)-y(3);-y(1)];
end

for j=1:3
    for i=1:30
        angle_leg=[angleright(:,i) angleleft(:,i)];
        angle_shoulder=zeros(1,10);
        angle2posnew(angle_leg,angle_shoulder);
    end
end

for k = 1:6
    s=[];
    for i=1:length(angleright(1,:))
        s=[s num2str(angleright(k,i)) ', '];
    end
    s=['a_RHipYawPitch = [' s ']'];
    disp(s);
end
for k = 1:6
    s=[];
    for i=1:length(angleleft(1,:))
        s=[s num2str(angleleft(k,i)) ', '];
    end
    s=['a_LHipYawPitch = [' s ']'];
    disp(s);
end