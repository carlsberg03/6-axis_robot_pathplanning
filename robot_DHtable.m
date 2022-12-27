function DHtable = robot_DHtable (~)
%robot_DHtable creates a DH table for an industrial robot.

% 完善函数robot_DHtable的注释，根据正运动学给出DH参数的值。

DHtable = [];

theta1 = 180/180*pi;
theta2 = -90/180*pi;
theta3 = 0;
theta4 = -90/180*pi;
theta5 = 0;
theta6 = 0;

d1 = 0.0985;
d2 = 0.1405;
d3 = 0;
d4 = -0.019;
d5 = 0.1025;
d6 = 0.094;

a1 = 0;
a2 = 0;
a3 = 0.408;
a4 = 0.376;
a5 = 0;
a6 = 0;

alpha1 = 0;
alpha2 = -90/180*pi;
alpha3 = 180/180*pi;
alpha4 = 180/180*pi;
alpha5 = -90/180*pi;
alpha6 = 90/180*pi;

DHtable = [theta1,d1,a1,alpha1;...
           theta2,d2,a2,alpha2;...
           theta3,d3,a3,alpha3;...
           theta4,d4,a4,alpha4;...
           theta5,d5,a5,alpha5;...
           theta6,d6,a6,alpha6];

end

