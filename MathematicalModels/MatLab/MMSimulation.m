addpath(genpath("./TrajGenerate"))
addpath(genpath("./Utils"))
% LitteRA的模型仿真
% Link函数的参数：关节角，横距，杆件长度，扭转角，关节类型：0代表旋转关节，非0代表移动关节
L(1) = Link([0, 0, 20, pi / 2]);
L(1).qlim = [-pi, pi];
L(2) = Link([0, 0, 105, 0]);
L(2).qlim = [0, pi];
L(3) = Link([0, 0, 95, -pi]);
L(3).qlim = [-pi / 2, pi / 2];
L(4) = Link([0, 0, 0, -pi]);
L(4).offset = -pi / 2;
L(4).qlim = [-pi / 2, pi / 2];
% 增加虚拟关节
L(5) = Link([0, 10, 0, pi / 2]);
L(5).qlim = [0, 0];
L(6) = Link([0, 60, 0, 0]);
LittleRA = SerialLink(L, 'name', 'LittleRA');
% 显示机械臂
% LittleRA.teach();
% 直线轨迹规划
% 输入参数分别为机器人模型，起始点，终止点，轨迹点的个数，运动时间
LineGenerate(LittleRA, transl(-100, 200, -50), transl(-200, -50, 75), 'unoriginal', 3, 50);

% 圆弧轨迹规划，至少三个输入点才能确定空间中的一个圆弧
% 输入参数分别为机器人模型，起始点，终止点，引导点，优化方式，轨迹点的个数
CircleGenerate(LittleRA, transl(-100, 200, -50), transl(-200, -50, 75), transl(-150, 100, -100), 3, 50);

% 角度空间轨迹规划
% 输入参数分别为机器人模型，起始点，终止点，轨迹点的个数
AngleGenerate(LittleRA, transl(-100, 200, -50), transl(-200, -50, 75), 'unoriginal', 3, 50);

