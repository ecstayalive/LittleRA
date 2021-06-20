function point = LineInterpol(StartPoint, EndPoint, degree, N)
    % 直线插补函数,输入优化级别，插补点个数，得到一组坐标
    % 脚本文件: LineInterpol.m
    % StartPoint: 起始点位置,齐次矩阵，包含有位姿与空间信息
    % EndPoint: 终止点位置,齐次矩阵，包含有位姿与空间信息
    % degree: 优化级别
    %       0: 不优化
    %       1: 匀加速匀减速优化(带抛物线过渡段)
    %       2: 三次多项式优化
    %       3: 五次多项式优化
    % N: 生成圆弧点的数量
    % point: 生成的点阵列

    StartPoint = transl(StartPoint); EndPoint = transl(EndPoint);
    % 直线参数方程
    x_1 = StartPoint(1); y_1 = StartPoint(2); z_1 = StartPoint(3);
    x_2 = EndPoint(1); y_2 = EndPoint(2); z_2 = EndPoint(3);
    % 获取优化后的时间序列
    kernel = Optimiza(1, degree, N);

    for i = 1:N
        x(i) = x_1 + (x_2 - x_1) * kernel(i);
        y(i) = y_1 + (y_2 - y_1) * kernel(i);
        z(i) = z_1 + (z_2 - z_1) * kernel(i);
        point(:, :, i) = transl(x(i), y(i), z(i));
    end

end
