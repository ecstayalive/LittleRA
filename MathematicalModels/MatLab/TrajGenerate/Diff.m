function [qd, qdd] = Diff(q, kind, degree, N)
    % 根据关节位置求解速度与加速度
    % 脚本文件: Diff.m
    % q: 关节位置
    % kind: 使用轨迹优化的方式
    % degree: 优化级别，只在kind=unorginal时起作用
    %       0: 不优化
    %       1: 匀加速匀减速优化(带抛物线过渡段)
    % N: 生成轨迹点的速度
    % [qd, qdd]速度和加速度
    for i = 1:length(q(:, 1))

        if (i == 1 || i == length(q(:, 1)))
            qd(i, :) = zeros(1, length(q(1, :)));
        else
            % 关节转角
            for j = 1:length(q(i, :))
                delta_ang(1, j) = q(i + 1, j) - q(i - 1, j);

                if (delta_ang(1, j) < -pi)
                    delta_ang(1, j) = delta_ang(1, j) + 2 * pi;
                elseif (delta_ang(1, j) > pi)
                    delta_ang(1, j) = delta_ang(1, j) - 2 * pi;
                end

            end

            qd(i, :) = delta_ang * (N - 1) / 2;
        end

    end

    % 求解关节加速度
    for i = 1:length(q(:, 1))

        if (i == 1)
            % 第一个点的加速度并不可信
            qdd(i, :) = zeros(1, length(q(1, :)));
        elseif (i == length(q(:, 1)))
            qdd(i, :) = qdd(i - 1, :);
        else
            qdd(i, :) = (qd(i + 1, :) - qd(i - 1, :)) * (N - 1) / 2;
        end

    end

    qdd(1, :) = qdd(2, :);
    % 如果是五次多项式优化按模型画图
    if (degree == 3 && kind == "unoriginal")
        qdd(1, :) = zeros(1, length(q(1, :)));
        qdd(length(q(:, 1)), :) = zeros(1, length(q(1, :)));
    end

end
