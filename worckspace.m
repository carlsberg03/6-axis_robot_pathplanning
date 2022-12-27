% workspace_computation
clear;
clc;

% load DHtable
figure(1)
hold on 
robot.plot([0 0 0 0 0 0], plotopt{:});
hold off
% angle limit for each joint
format short;
num=10000;%仿真点数
limit = ones(6,2);
limit(:,1)=-deg2rad(178);
limit(:,2)=deg2rad(178);

% set step for worskpace computation
step = 10;
% TODO
q1_rand = limit(1,1) + rand(num,1)*(limit(1,2) - limit(1,1));
q2_rand = limit(2,1) + rand(num,1)*(limit(2,2) - limit(2,1));
q3_rand = limit(3,1) + rand(num,1)*(limit(3,2) - limit(3,1));
q4_rand = limit(4,1) + rand(num,1)*(limit(4,2) - limit(4,1));
q5_rand = limit(5,1) + rand(num,1)*(limit(5,2) - limit(5,1));
q6_rand = rand(num,1)*0;
q = [q1_rand q2_rand q3_rand q4_rand q5_rand q6_rand];

% compute workspace
tic;
% TODO
 T_cell = cell(num,1);
 [T_cell{:,1}]=robot.fkine(q).t;%正向运动学仿真函数
 disp(['运行时间：',num2str(toc)]);
figure('name','机械臂工作空间')
    hold on
    plotopt = {'noraise', 'nowrist', 'nojaxes', 'delay',0};
    robot.plot([0 0 0 0 0 0], plotopt{:});
     figure_x=zeros(num,1);
     figure_y=zeros(num,1);
     figure_z=zeros(num,1);
     for cout=1:1:num
         figure_x(cout,1)=T_cell{cout}(1);
         figure_y(cout,1)=T_cell{cout}(2);
         figure_z(cout,1)=T_cell{cout}(3);
     end
     plot3(figure_x,figure_y,figure_z,'r.','MarkerSize',3);
     hold off
t0=toc;
disp(['Computational time: ',num2str(t0)]); 




