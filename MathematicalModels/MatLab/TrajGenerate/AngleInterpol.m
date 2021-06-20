function q = AngleInterpol(QO, QF, degree, N)
    % 角度空间插补函数,输入优化级别，插补点个数，得到一组坐标
    % 脚本文件: AngleInterpol.m
    % QO: 初始关节角度,1XN
    % QF: 终止点位置,1XN
    % degree: 优化级别
    %       0: 不优化
    %       1: 匀加速匀减速优化(带抛物线过渡段)
    %       2: 三次多项式优化
    %       3: 五次多项式优化
    % N: 生成圆弧点的数量
    % q: 生成的角度阵列

    % 变换角度
    Delta_ang = QF - QO;
    % 获取三个关节角度的轨迹优化阵列
    for i = 1:length(Delta_ang)
        kernel(:, i) = Optimiza(Delta_ang(i), degree, N);
        q(:, i) = kernel(:, i) + QO(i) * ones(N, 1);
    end

end
