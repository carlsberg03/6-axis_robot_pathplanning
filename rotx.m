function T = rotx(s)
% 代码补充完整
a = s;
T = [1,0,0,0;...
    0,cos(a),-sin(a),0;...
    0,sin(a),cos(a),0;...
    0,0,0,1];
end