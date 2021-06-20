function CircleGenerate(Robot, StartPoint, LeadPoint, EndPoint, degree, N)
    % 机器人圆弧路径规划，至少三个输入点才能确定空间的圆弧
    % 脚本文件: CircleGenerate.m
    % Robot: 机器人模型
    % StartPoint: 机器人运动起始点   四维齐次变换矩阵
    % LeadPoint: 机器人运动圆弧引导点  四维齐次变换矩阵
    % EndPoint: 机器人运动结束点     四维齐次变换矩阵
    % degree: 优化级别
    %       0: 不优化
    %       1: 匀加速匀减速优化(带抛物线过渡段)
    %       2: 三次多项式优化
    %       3: 五次多项式优化
    % N: 笛卡尔空间轨迹规划步数

    try
        warning('off');
        % 步数矩阵
        point = CirInterpol(StartPoint, LeadPoint, EndPoint, degree, N);
        % 通过空间位置逆运动学求解关节角度轨迹
        q = Robot.ikine(point, 'mask', [1, 1, 1, 0, 0, 0]);
        [qd, qdd] = Diff(q, 'unoriginal', degree, N);
        grid on

        % 正运动学
        T = Robot.fkine(q);
        T = transl(T);
        % 关节位置
        subplot(3, 2, 1);
        i = 1:length(q(1, :));
        plot(q(:, i));
        title('Position');
        grid on
        % 关节速度
        subplot(3, 2, 3);
        i = 1:length(qd(1, :));
        plot(qd(:, i));
        title('Speed');
        grid on
        % 关节加速度
        subplot(3, 2, 5);
        i = 1:length(qdd(1, :));
        plot(qdd(:, i));
        title('Acceleration');
        grid on
        % 动画演示
        subplot(3, 2, [2, 4, 6]);
        % 输出末端轨迹
        plot3(T(:, 1), T(:, 2), T(:, 3));
        hold on
        Robot.plot(q);
    catch
        fprintf('迭代误差过大，位姿解算错误，无法达到轨迹中的某些点\n');
    end

end
