function PlotWorkspace(Robot, PointNum)
    % 画出机器人可达空间散点图
    % 脚本文件: PlotWorkspace.m
    % Robot: 建立的机器人模型
    % PointNum: 工作空间随机点的个数

    % 三个关节变量限位
    A = unifrnd(-pi, pi, [1, PointNum]);
    B = unifrnd(0, pi, [1, PointNum]);
    C = unifrnd(-pi / 2, pi / 2, [1, PointNum]);
    D = unifrnd(-pi / 2, pi / 2, [1, PointNum]);
    E = unifrnd(-pi, pi, [1, PointNum]);
    % 建立元胞数组
    G = cell(PointNum, 5);
    % 产生PointNum组随机点
    for n = 1:PointNum
        G{n} = [A(n) B(n) C(n) D(n) E(n)];
    end

    % 将元胞数组转化为矩阵
    H1 = cell2mat(G);
    % 机械臂正解
    T = double(Robot.fkine(H1));
    % figure(1)
    % 随机点图
    scatter3(squeeze(T(1, 4, :)), squeeze(T(2, 4, :)), squeeze(T(3, 4, :)));
    % 机械臂图
    Robot.plot([pi / 2 pi / 4 0 0 0], 'floorlevel', 0);
    hold on;

end
