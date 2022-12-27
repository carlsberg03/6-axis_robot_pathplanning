function T = trans(a,b,c)
% trans 用于生成机器人的平移齐次变换矩阵
% 输入参数：
%   a,b,c: 分别沿着x,y,z轴的移动量，单位(mm)
% 输出参数：
%   T：4x4齐次变换矩阵

% 版本号V1.0，编写于2022.10.1，修改于2022.10.31，作者：Chen

% 函数主体
% 判断输入变量的数量
if nargin<1
    a=0;b=0;c=0;
elseif nargin<2
    b=0;c=0;
elseif nargin<3
    c=0;
elseif nargin > 3                                                  
    error('输入变量过多！');
end

% 根据平移齐次变换矩阵，定义矩阵
T = [1,0,0,a;0,1,0,b;0,0,1,c;0,0,0,1];

end
