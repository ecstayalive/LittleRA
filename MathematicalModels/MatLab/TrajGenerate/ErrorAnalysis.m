function q = ErrorAnalysis(Robot, point, N)
    % 圆弧插补函数，输入三个坐标，以及插补点个数，得到一组坐标
    % 脚本文件: ErrorAnalysis.m
    % Robot: 机器人模型
    % point: 生成的轨迹中的三维点
    % N: 生成圆弧点的数量

    try
        fprintf("开始错误分析，可能需要一些时间\n")

        for i = 1:N
            q(1:i, :) = Robot.ikine(point(:, :, 1:i), 'mask', [1, 1, 1, 0, 0, 0]);
        end

    catch
        fprintf("错误分析完成\n");

        if i <= 2
            fprintf("初始点不可达\n");
        else
            fprintf("规划点中第%d点与第%d为奇异点附近的点，两点之间无法实现关节角连续可达\n", i - 1, i);
        end

    end

end
