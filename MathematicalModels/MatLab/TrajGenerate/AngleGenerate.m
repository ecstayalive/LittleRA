function AngleGenerate(Robot, StartPoint, EndPoint, kind, degree, N)
    % 机器人角度空间路径规划
    % 脚本文件: AngleGenerate.m
    % Robot: 机器人模型
    % StartPoint: 机器人运动起始点 四维齐次变换矩阵
    % EndPoint: 机器人运动结束点   四维齐次变换矩阵
    % kind: 角度空间规划的方式
    % degree: 优化级别
    %       0: 不优化
    %       1: 匀加速匀减速优化(带抛物线过渡段)
    %       2: 三次多项式优化
    %       3: 五次多项式优化
    % N: 采样点个数

    try
        warning('off');
        % 逆运动学，得到起始和终止关节角度
        q1 = Robot.ikine(StartPoint, 'mask', [1, 1, 1, 0, 0, 0]);
        q2 = Robot.ikine(EndPoint, 'mask', [1, 1, 1, 0, 0, 0]);
        % 五次多项式轨迹，得到关节角度，角速度，角加速度，采样点个数为50
        switch kind
            case 'original'
                [q, qd, qdd] = jtraj(q1, q2, N);
            case 'unoriginal'
                q = AngleInterpol(q1, q2, degree, N);
                [qd, qdd] = Diff(q, kind, degree, N);

        end

        grid on
        % 根据插值，得到末端执行器位姿
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
