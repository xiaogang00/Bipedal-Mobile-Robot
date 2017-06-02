%%输入舵机的角度，可以得到整个机器人姿态的函数
function angle2posnew(angle_leg,angle_shoulder)
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
au=[0 0 0 1 0 1 0 0 1 0 1;
    0 1 0 0 0 0 1 0 0 0 0;
    1 0 1 0 1 0 0 1 0 1 0];
bu=[0 0 0 UpperArmLength 0 LowerArmLength 0 0 UpperArmLength 0 LowerArmLength;
    0 -ShoulderOffsetY 0 0 0 0 ShoulderOffsetY 0 0 0 0;
    NeckOffsetZ+HipOffsetZ 0 0 0 0 0 0 0 0 0 0];
%%
RHipYawPitch=angle_leg(1);      %已经按照解的顺序修改过
RHipRoll=angle_leg(2);
RHipPitch=angle_leg(3);
RKneePitch=angle_leg(4);
RAnklePitch=angle_leg(5);
RAnkleRoll=angle_leg(6);
LHipYawPitch=angle_leg(7);
LHipRoll=angle_leg(8);
LHipPitch=angle_leg(9);
LKneePitch=angle_leg(10);
LAnklePitch=angle_leg(11);
LAnkleRoll=angle_leg(12);

RShoulderPitch=angle_shoulder(1);
RShoulderRoll=angle_shoulder(2);
REllbowYaw=angle_shoulder(3);
REllbowRoll=angle_shoulder(4);
RWingYaw=angle_shoulder(5);
LShoulderPitch=angle_shoulder(6);
LShoulderRoll=angle_shoulder(7);
LEllbowYaw=angle_shoulder(8);
LEllbowRoll=angle_shoulder(9);
LWingYaw=angle_shoulder(10);
%% state-transition matrix for leg
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
t1=T1(0);
t2=T2(RHipYawPitch);
t3=T3(RHipRoll);
t4=T4(RHipPitch);
t5=T5(RKneePitch);
t6=T6(RAnklePitch);
t7=T7(RAnkleRoll);
t8=T8(LHipYawPitch);
t9=T9(LHipRoll);
t10=T10(LHipPitch);
t11=T11(LKneePitch);
t12=T12(LAnklePitch);
t13=T13(LAnkleRoll);
point0=t1*[0;0;0;1];
point1=t1*[0;-HipOffsetY;0;1];
point2=t1*t2*t3*t4*[0;0;-ThighLength;1];
point3=t1*t2*t3*t4*t5*[0;0;-TibiaLength;1];
point4=t1*t2*t3*t4*t5*t6*t7*[0;0;-FootHeight;1];
point5=t1*[0;HipOffsetY;0;1];
point6=t1*t8*t9*t10*[0;0;-ThighLength;1];
point7=t1*t8*t9*t10*t11*[0;0;-TibiaLength;1];
point8=t1*t8*t9*t10*t11*t12*t13*[0;0;-FootHeight;1];
Rfootfront=t1*t2*t3*t4*t5*t6*t7*[75;0;-FootHeight;1];
Lfootfront=t1*t8*t9*t10*t11*t12*t13*[75;0;-FootHeight;1];
Rf1=t1*t2*t3*t4*t5*t6*t7*[75;-20;-FootHeight;1];
Rf2=t1*t2*t3*t4*t5*t6*t7*[0;-20;-FootHeight;1];
Rf3=t1*t2*t3*t4*t5*t6*t7*[0;20;-FootHeight;1];
Rf4=t1*t2*t3*t4*t5*t6*t7*[75;20;-FootHeight;1];
Lf1=t1*t8*t9*t10*t11*t12*t13*[75;-20;-FootHeight;1];
Lf2=t1*t8*t9*t10*t11*t12*t13*[0;-20;-FootHeight;1];
Lf3=t1*t8*t9*t10*t11*t12*t13*[0;20;-FootHeight;1];
Lf4=t1*t8*t9*t10*t11*t12*t13*[75;20;-FootHeight;1];
%% state-transition matrix for leg
Tu1=@(x)([[eye(3)+[0 -au(3,1) au(2,1);au(3,1) 0 -au(1,1);-au(2,1) au(1,1) 0]*sin(x)+[0 -au(3,1) au(2,1);au(3,1) 0 -au(1,1);-au(2,1) au(1,1) 0]*[0 -au(3,1) au(2,1);au(3,1) 0 -au(1,1);-au(2,1) au(1,1) 0]*(1-cos(x))],bu(:,1);0 0 0 1]);
Tu2=@(x)([[eye(3)+[0 -au(3,2) au(2,2);au(3,2) 0 -au(1,2);-au(2,2) au(1,2) 0]*sin(x)+[0 -au(3,2) au(2,2);au(3,2) 0 -au(1,2);-au(2,2) au(1,2) 0]*[0 -au(3,2) au(2,2);au(3,2) 0 -au(1,2);-au(2,2) au(1,2) 0]*(1-cos(x))],bu(:,2);0 0 0 1]);
Tu3=@(x)([[eye(3)+[0 -au(3,3) au(2,3);au(3,3) 0 -au(1,3);-au(2,3) au(1,3) 0]*sin(x)+[0 -au(3,3) au(2,3);au(3,3) 0 -au(1,3);-au(2,3) au(1,3) 0]*[0 -au(3,3) au(2,3);au(3,3) 0 -au(1,3);-au(2,3) au(1,3) 0]*(1-cos(x))],bu(:,3);0 0 0 1]);
Tu4=@(x)([[eye(3)+[0 -au(3,4) au(2,4);au(3,4) 0 -au(1,4);-au(2,4) au(1,4) 0]*sin(x)+[0 -au(3,4) au(2,4);au(3,4) 0 -au(1,4);-au(2,4) au(1,4) 0]*[0 -au(3,4) au(2,4);au(3,4) 0 -au(1,4);-au(2,4) au(1,4) 0]*(1-cos(x))],bu(:,4);0 0 0 1]);
Tu5=@(x)([[eye(3)+[0 -au(3,5) au(2,5);au(3,5) 0 -au(1,5);-au(2,5) au(1,5) 0]*sin(x)+[0 -au(3,5) au(2,5);au(3,5) 0 -au(1,5);-au(2,5) au(1,5) 0]*[0 -au(3,5) au(2,5);au(3,5) 0 -au(1,5);-au(2,5) au(1,5) 0]*(1-cos(x))],bu(:,5);0 0 0 1]);
Tu6=@(x)([[eye(3)+[0 -au(3,6) au(2,6);au(3,6) 0 -au(1,6);-au(2,6) au(1,6) 0]*sin(x)+[0 -au(3,6) au(2,6);au(3,6) 0 -au(1,6);-au(2,6) au(1,6) 0]*[0 -au(3,6) au(2,6);au(3,6) 0 -au(1,6);-au(2,6) au(1,6) 0]*(1-cos(x))],bu(:,6);0 0 0 1]);
Tu7=@(x)([[eye(3)+[0 -au(3,7) au(2,7);au(3,7) 0 -au(1,7);-au(2,7) au(1,7) 0]*sin(x)+[0 -au(3,7) au(2,7);au(3,7) 0 -au(1,7);-au(2,7) au(1,7) 0]*[0 -au(3,7) au(2,7);au(3,7) 0 -au(1,7);-au(2,7) au(1,7) 0]*(1-cos(x))],bu(:,7);0 0 0 1]);
Tu8=@(x)([[eye(3)+[0 -au(3,8) au(2,8);au(3,8) 0 -au(1,8);-au(2,8) au(1,8) 0]*sin(x)+[0 -au(3,8) au(2,8);au(3,8) 0 -au(1,8);-au(2,8) au(1,8) 0]*[0 -au(3,8) au(2,8);au(3,8) 0 -au(1,8);-au(2,8) au(1,8) 0]*(1-cos(x))],bu(:,8);0 0 0 1]);
Tu9=@(x)([[eye(3)+[0 -au(3,9) au(2,9);au(3,9) 0 -au(1,9);-au(2,9) au(1,9) 0]*sin(x)+[0 -au(3,9) au(2,9);au(3,9) 0 -au(1,9);-au(2,9) au(1,9) 0]*[0 -au(3,9) au(2,9);au(3,9) 0 -au(1,9);-au(2,9) au(1,9) 0]*(1-cos(x))],bu(:,9);0 0 0 1]);
Tu10=@(x)([[eye(3)+[0 -au(3,10) au(2,10);au(3,10) 0 -au(1,10);-au(2,10) au(1,10) 0]*sin(x)+[0 -au(3,10) au(2,10);au(3,10) 0 -au(1,10);-au(2,10) au(1,10) 0]*[0 -au(3,10) au(2,10);au(3,10) 0 -au(1,10);-au(2,10) au(1,10) 0]*(1-cos(x))],bu(:,10);0 0 0 1]);
Tu11=@(x)([[eye(3)+[0 -au(3,11) au(2,11);au(3,11) 0 -au(1,11);-au(2,11) au(1,11) 0]*sin(x)+[0 -au(3,11) au(2,11);au(3,11) 0 -au(1,11);-au(2,11) au(1,11) 0]*[0 -au(3,11) au(2,11);au(3,11) 0 -au(1,11);-au(2,11) au(1,11) 0]*(1-cos(x))],bu(:,11);0 0 0 1]);
tu1=Tu1(0);
tu2=Tu2(RShoulderPitch);
tu3=Tu3(RShoulderRoll);
tu4=Tu4(REllbowYaw);
tu5=Tu5(REllbowRoll);
tu6=Tu6(RWingYaw);
tu7=Tu7(LShoulderPitch);
tu8=Tu8(LShoulderRoll);
tu9=Tu9(LEllbowYaw);
tu10=Tu10(LEllbowRoll);
tu11=Tu11(LWingYaw);
point00=tu1*[0;0;0;1];
point01=tu1*[0;-ShoulderOffsetY;0;1];
point02=tu1*tu2*tu3*[UpperArmLength;0;0;1];
point03=tu1*tu2*tu3*tu4*tu5*[LowerArmLength;0;0;1];
point04=tu1*tu2*tu3*tu4*tu5*tu6*[HandOffsetX;0;0;1];
point05=tu1*[0;ShoulderOffsetY;0;1];
point06=tu1*tu7*tu8*[UpperArmLength;0;0;1];
point07=tu1*tu7*tu8*tu9*tu10*[LowerArmLength;0;0;1];
point08=tu1*tu7*tu8*tu9*tu10*tu11*[HandOffsetX;0;0;1];
pointhead=point0+[0 0 250 1]';
pause(5)
%% plot figure
pointset=[point0 point1 point2 point3 point4 point5 point6 point7 point8 point00 point01 point02 point03 point04 point05 point06 point07 point08 pointhead Rfootfront Rf1 Rf2 Rf3 Rf4 Lfootfront Lf1 Lf2 Lf3 Lf4];
pointset=pointset';
point=pointset(:,1:3);
plotrobot(point)
pause(0.1)
