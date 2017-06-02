function plotrobot(pointset)
scatter3(pointset(:,1),pointset(:,2),pointset(:,3))
hold on
Array1=[pointset(1,:);pointset(2,:);pointset(3,:);pointset(4,:);pointset(5,:)];
Array2=[pointset(1,:);pointset(6,:);pointset(7,:);pointset(8,:);pointset(9,:)];
Array3=[pointset(1,:);pointset(10,:);pointset(11,:);pointset(12,:);pointset(13,:);pointset(14,:)];
Array4=[pointset(1,:);pointset(10,:);pointset(15,:);pointset(16,:);pointset(17,:);pointset(18,:)];
Array5=[pointset(1,:);pointset(19,:)];
Array6=[pointset(5,:);pointset(20,:);pointset(21,:);pointset(22,:);pointset(23,:);pointset(24,:)];
Array7=[pointset(9,:);pointset(25,:);pointset(26,:);pointset(27,:);pointset(28,:);pointset(29,:)];
plot3(Array1(:,1),Array1(:,2),Array1(:,3));
axis([-100 500 -300 300 -300 300]);
hold on
plot3(Array2(:,1),Array2(:,2),Array2(:,3));
hold on
plot3(Array3(:,1),Array3(:,2),Array3(:,3));
hold on
plot3(Array4(:,1),Array4(:,2),Array4(:,3));
hold on
plot3(Array5(:,1),Array5(:,2),Array5(:,3));
hold on
plot3(Array6(:,1),Array6(:,2),Array6(:,3));
hold on
plot3(Array7(:,1),Array7(:,2),Array7(:,3));
hold off
view(45,30)
%% calculate foot
% n1=pointset(5,:)-pointset(4,:);
% n2=pointset(9,:)-pointset(8,:);
% syms x z;
% [ans1 ans2]=solve('n1(1)*x+n1(2)*pointset(5,2)+n1(3)*z-n1(1)*pointset(5,1)-n1(2)*pointset(5,2)-n1(3)*pointset(5,3)','(x-pointset(5,1)^2+(z-pointset(5,3)^2=50^2))');
