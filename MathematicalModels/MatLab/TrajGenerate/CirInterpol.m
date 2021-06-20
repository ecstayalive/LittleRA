function point = CirInterpol(StartPoint, EndPoint, LeadPoint, degree, N)
    % 圆弧插补函数，输入三个坐标，以及插补点个数，得到一组坐标
    % 原理详解: https://blog.csdn.net/qq_26565435/article/details/96972935
    % 脚本文件: CirInterpol.m
    % StartPoint: 起始点位置,齐次矩阵，包含有位姿与空间信息
    % EndPoint: 终止点位置,齐次矩阵，包含有位姿与空间信息
    % LeadinPoint: 引导点位置,齐次矩阵，包含有位姿与空间信息
    % degree: 优化级别
    %       0: 不优化
    %       1: 匀加速匀减速优化(带抛物线过渡段)
    %       2: 三次多项式优化
    %       3: 五次多项式优化
    % N: 生成圆弧点的数量
    % point: 生成的点阵列

    StartPoint = transl(StartPoint); LeadPoint = transl(LeadPoint); EndPoint = transl(EndPoint);

    x1 = StartPoint(1); x2 = LeadPoint(1); x3 = EndPoint(1);
    y1 = StartPoint(2); y2 = LeadPoint(2); y3 = EndPoint(2);
    z1 = StartPoint(3); z2 = LeadPoint(3); z3 = EndPoint(3);

    A1 = (y1 - y3) * (z2 - z3) - (y2 - y3) * (z1 - z3);
    B1 = (x2 - x3) * (z1 - z3) - (x1 - x3) * (z2 - z3);
    C1 = (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
    D1 = -(A1 * x3 + B1 * y3 + C1 * z3);

    A2 = x2 - x1;
    B2 = y2 - y1;
    C2 = z2 - z1;
    D2 = -((x2^2 - x1^2) + (y2^2 - y1^2) + (z2^2 - z1^2)) / 2;

    A3 = x3 - x2;
    B3 = y3 - y2;
    C3 = z3 - z2;
    D3 = -((x3^2 - x2^2) + (y3^2 - y2^2) + (z3^2 - z2^2)) / 2;
    A = [A1, B1, C1; A2, B2, C2; A3, B3, C3];
    b = [-D1; -D2; -D3];
    % 通过高斯主元消去法求得圆心
    % C = GaussLie(3, A, b);
    % 通过矩阵逆运算求得圆心
    C = A^ - 1 * b;
    x0 = C(1); y0 = C(2); z0 = C(3);
    % plot3(x0, y0, z0, 'bo');
    hold on;
    % 外接圆半径
    r = sqrt(power(x1 - x0, 2) + power(y1 - y0, 2) + power(z1 - z0, 2));
    % 新坐标系Z0的方向余弦
    L = sqrt(A1^2 + B1^2 + C1^2);
    % 新坐标系为三点确定平面的法向量
    ax = A1 / L; ay = B1 / L; az = C1 / L;
    % 新坐标系X轴的方向为StartPoint与圆心的方向向量
    nx = (x1 - x0) / r;
    ny = (y1 - y0) / r;
    nz = (z1 - z0) / r;
    % 新坐标系Y0的方向余弦
    o = cross([ax, ay, az], [nx, ny, nz]);
    ox = o(1);
    oy = o(2);
    oz = o(3);
    % 相对于基座标系{O-XYZ}， 新坐标系{C-X0Y0Z0}的坐标变换矩阵
    T = [nx ox ax x0;
        ny oy ay y0;
        nz oz az z0;
        0 0 0 1];
    % 求在新坐标系{C-X0Y0Z0}下StartPoint、LeadPoint和EndPoint的坐标
    S_ = (T^ - 1) * [StartPoint; 1];
    M_ = (T^ - 1) * [LeadPoint; 1];
    D_ = (T^ - 1) * [EndPoint; 1];
    x1_ = S_(1); y1_ = S_(2); z1_ = S_(3);
    x2_ = M_(1); y2_ = M_(2); z2_ = M_(3);
    x3_ = D_(1); y3_ = D_(2); z3_ = D_(3);
    % 判断圆弧是顺时针还是逆时针，并求解圆心角
    theta = atan2(y3_, x3_);
    % 插补原理: 在新平面上进行插补（简化）
    % 在新坐标系下z1_,z2_,z3_均为0，即外接圆在新坐标系的XOY平面内
    % 此时转化为平面圆弧插补问题
    kernel = Optimiza(theta, degree, N);

    for i = 1:N
        x_(i) = r * cos(kernel(i)); y_(i) = r * sin(kernel(i));
        P = T * [x_(i); y_(i); 0; 1];
        x(i) = P(1); y(i) = P(2); z(i) = P(3);
        point(:, :, i) = transl(x(i), y(i), z(i));
    end

end
