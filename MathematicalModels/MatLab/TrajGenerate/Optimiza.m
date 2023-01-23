function kernel = Optimiza(theta, degree, N)
    % 轨迹优化
    % 轨迹起点与终点加速度无穷大，或者突变，容易对机械臂关节造成伤害
    % 脚本文件: Optimiza.m
    % theta: 变化值
    % degree: 优化级别
    %       0: 不优化
    %       1: 匀加速匀减速优化(带抛物线过渡段)
    %       2: 三次多项式优化
    %       3: 五次多项式优化
    % N: 优化后轨迹阵列的点的数量
    % kernel: 优化后轨迹阵列

    switch degree
        case 0

            for i = 1:N
                kernel(i) = (i - 1) * theta / (N - 1);
            end

        case 1
            % 匀加速匀减速优化(带抛物线过渡段)
            if (theta < 0)
                flag = -1;
            else
                flag = 1;
            end

            theta = abs(theta);
            % 指定加速度
            psi = 16 * theta;
            % 指定加速时间
            t_b = 0.5 - sqrt(psi ^ 2 - 4 * psi * theta) / (2 * psi);

            for i = 1:N
                % 时刻t
                t = (i - 1) / (N - 1);

                if (t < t_b)
                    kernel(i) = psi * t ^ 2/2;
                elseif (t >= t_b && t <= 1 - t_b)
                    kernel(i) = psi * t_b * t - psi * t_b ^ 2/2;
                else
                    kernel(i) = theta - psi * (1 - t) ^ 2/2;
                end

            end

            kernel = flag * kernel;

        case 2
            % 三次多项式优化
            for i = 1:N
                % 时刻t
                t = (i - 1) / (N - 1);
                kernel(i) = 3 * theta * t ^ 2 - 2 * theta * t ^ 3;
            end

        case 3
            % 五次多项式优化
            for i = 1:N
                % 时刻t
                t = (i - 1) / (N - 1);
                kernel(i) = 10 * theta * t ^ 3 - 15 * theta * t ^ 4 + 6 * theta * t ^ 5;
            end

    end

end
