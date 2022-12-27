function [H, H_i] = robot_fkin(DHtable,q)
% robot_fkin根据DH表和关节角度q生成末端到基座的齐次变换矩阵H和相邻连杆的齐次变换矩阵H_i
% 输入参数：
%   DHtable是一个nX4 的矩阵，满足DH改进约定的DH表，可从robot_DHtable()中读取；
%   q是一个1xN的向量，表示关节坐标
% 输出参数：
%   H表示末端到基座的 4X4 齐次变换矩阵；
%   H_i表示相邻连杆的 4X4 齐次变换矩阵，由于有n个连杆，因此H_i应该为包含n个元素的cell；

num_link = size(DHtable,1);
H = eye(4);
H_i = cell(1,num_link);

for i = 1:num_link
    H_i{1,i} = rotx(DHtable(i,4))*trans(DHtable(i,3),0,0)...
        *rotz(DHtable(i,1)+q(i))*trans(0,0,DHtable(i,2));
    H = H*H_i{1,i};
end

end
