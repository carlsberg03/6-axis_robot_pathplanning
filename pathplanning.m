%% 清空工作区
clear;
close all;
clc;
%% 读取DH参数
DHtable = robot_DHtable();
L(1)=Link([DHtable(1,:),0],'modified');
L(2)=Link([DHtable(2,:),0],'modified');
L(3)=Link([DHtable(3,:),0],'modified');
L(4)=Link([DHtable(4,:),0],'modified');
L(5)=Link([DHtable(5,:),0],'modified');
L(6)=Link([DHtable(6,:),0],'modified');
%s为机械臂初始姿态
s=[pi -pi/2 0 -pi/2 0 0];
plotopt = {'noraise', 'nowrist', 'nojaxes', 'delay',0};
robot=SerialLink(L,'name','robot');
%% 步长和采样次数，请确保足够大的采样次数以获得正确结果

%如果运行报错，可以尝试增大samples或者重新运行
samples = 20000;
segmentlength = 0.02;
%% 读取input.txt
fid=fopen("input.txt");
init = (fgetl(fid));
init = str2num(init);
target = fgetl(fid);
target = str2num(target);
obstacles_num = str2num(fgetl(fid));
obstacles=[];
for i = 1:obstacles_num
    temp=fgetl(fid);
    temp=str2num(temp);
    obstacles=[obstacles;temp];
end
fclose(fid);

%% 获取路径
[path , tree] =  RRT(init,target,obstacles_num,obstacles,samples,segmentlength);
q = path(:,1:6);
for i = 1:size(q,1)
    q(i,:)=q(i,:)+s;
end

T=robot.fkine(q);
num = size(path,1);
TJ=zeros(4,4,num);

%% 画图
figure(1);
clf;hold on;axis equal;
xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);
for i = 1:obstacles_num
    h(i)=drawSphere(obstacles(i,:)); % 画圆，用到geom3d工具包
end

robot.plot(init + s); % 画出初始位姿

% 计算每个关节坐标下的齐次变换矩阵
for ii = 1:num
    TJ(:,:,ii)=T(ii).T;
end


plot3(squeeze(TJ(1,4,:)),squeeze(TJ(2,4,:)),squeeze(TJ(3,4,:)));%输出末端轨迹
robot.plot(q) % 输出运动动画


%% RRT函数
function [paths,tree] =  RRT(init,target,NumObstacles,Obstacles,samples,segmentLength)

    dim=6;
    start_cord = init;
    goal_cord = target;


world = createWorld(NumObstacles,Obstacles);

start_node = [start_cord,0,0,0];
end_node = [goal_cord,0,0,0];
paths=[];
% 建立树结构
tree = start_node;
numPaths = 0;
a = clock;
% 检查初始节点和目标节点能否直接相连
if ( (norm(start_node(1:dim)-end_node(1:dim))<segmentLength )...
        &&(collision(start_node,world)==0) )
    paths = [start_node; end_node];
else
        for i = 1:samples
            flag = 0;
            [tree,flag] = extendTree(tree,end_node,segmentLength,world,flag,dim);
            numPaths = numPaths + flag;
            search=i
        end
end

% find path with minimum cost to end_node
if(numPaths>0)
paths = findMinimumPath(tree,end_node,dim);
end
end





function world = createWorld(NumObstacles, Obstacles)

        world.NumObstacles = NumObstacles;
        world.endcorner = [pi pi pi pi pi pi ];
        world.origincorner = [-pi -pi -pi -pi -pi -pi];
        
        for i=1:NumObstacles
            world.cx(i) = Obstacles(i,1);
            world.cy(i) = Obstacles(i,2);
            world.cz(i) = Obstacles(i,3);
            world.radius(i) = Obstacles(i,4);
        end
end




%% 碰撞检测
function collision_flag = collision(node, world)
collision_flag = 0;
dim = 6;

for i=1:dim
    if (node(i)>world.endcorner(i))||(node(i)<world.origincorner(i))
        collision_flag = 1;
    end
end

if collision_flag == 0
    DHtable = robot_DHtable();
    [~, H_i] = robot_fkin(DHtable,node);
    pos=zeros(6,3);
    H{1}=H_i{1,1};
    H{2}=H_i{1,1}*H_i{1,2};
    H{3}=H_i{1,1}*H_i{1,2}*H_i{1,3};
    H{4}=H_i{1,1}*H_i{1,2}*H_i{1,3}*H_i{1,4};
    H{5}=H_i{1,1}*H_i{1,2}*H_i{1,3}*H_i{1,4}*H_i{1,5};
    H{6}=H_i{1,1}*H_i{1,2}*H_i{1,3}*H_i{1,4}*H_i{1,5}*H_i{1,6};
    
    for i=1:6
        pos(i,:)=(H{i}(1:3,4))';
    end
    for j=1:world.NumObstacles
        for k=1:6
            if k==1
                x1=[0 0 0];
            else
                x1=pos(k-1,:);
            end
            x2=pos(k,:);
            pt=[world.cx(j),world.cy(j),world.cz(j)];

            if(dot(pt-x1,x1-x2)*dot(pt-x2,x1-x2)<0)
                if(norm(cross((pt-x1),(pt-x2)))/norm(x2-x1)<world.radius(j)+0.1)
                    collision_flag = 1;
  
                end
            else
                if(norm(pt-x1)<world.radius(j)||norm(pt-x1)<world.radius(j)+0.1)
                    collision_flag =1;
                end
            end
        end
    end
end
end



%% 树结构扩展
function [new_tree,flag] = extendTree(tree,end_node,segmentLength,world,flag_chk,dim)

flag1 = 0;
while flag1==0
    % 在区域内随机选一个点
    randomPoint = ones(1,dim);
    if rand>0.9
        randomPoint=end_node(1:6);
    else
    for i=1:dim
        randomPoint(1,i) = world.origincorner(i)+(world.endcorner(i)-world.origincorner(i))*rand; 
    end 
    end
    tmp = tree(:,1:dim)-ones(size(tree,1),1)*randomPoint;
    sqrd_dist = sqr_eucl_dist(tmp,dim); %返回各点到随机点直线距离的平方值
    [~,idx] = min(sqrd_dist);
    new_point = (randomPoint-tree(idx,1:dim));
    if(norm(new_point)<segmentLength)
        new_point=randomPoint;
    else 
        new_point = tree(idx,1:dim)+(new_point/norm(new_point))*segmentLength;    
    end
    min_cost  = cost_np(tree(idx,:),new_point,dim);
    new_node  = [new_point, 0, min_cost, idx];
    collision_flag=collision(new_node, world);
    if collision_flag==0
        new_tree = [tree; new_node];
        flag1=1;
    end
end

if flag_chk == 0
    % check to see if new node connects directly to end_node
    if ( (norm(new_node(1:dim)-end_node(1:dim))<segmentLength )...
            && (collision(new_node,world)==0) )
        flag = 1;
        new_tree(end,dim+1)=1;  % mark node as connecting to end.
    else
        flag = 0;
    end
    
else
    flag = 1;
end
end

%%
function e_dist = sqr_eucl_dist(array,dim)

sqr_e_dist = zeros(size(array,1),dim);
for i=1:dim
    
    sqr_e_dist(:,i) = array(:,i).*array(:,i);
    
end
e_dist = zeros(size(array,1),1);
for i=1:dim
    
    e_dist = e_dist+sqr_e_dist(:,i);
    
end

end



%calculate the cost from a node to a point
function [cost] = cost_np(from_node,to_point,dim)

diff = from_node(:,1:dim) - to_point;
eucl_dist = norm(diff);
cost = from_node(:,dim+2) + eucl_dist;

end


%%
function path = findMinimumPath(tree,end_node,dim)

% find nodes that connect to end_node
connectingNodes = [];
for i=1:size(tree,1)
    if tree(i,dim+1)==1
        connectingNodes = [connectingNodes ; tree(i,:)];
    end
end

if size(connectingNodes, 1) > 0

    % find minimum cost last node
    [~,idx] = min(connectingNodes(:,dim+2));
    
    % construct lowest cost path
    path = [connectingNodes(idx,:); end_node];
    parent_node = connectingNodes(idx,dim+3);
    while parent_node>1,
        parent_node = tree(parent_node,dim+3);
        path = [tree(parent_node,:); path];
    end
    
else
    path = [];
end

end

