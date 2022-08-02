function A = IKmatrix_manual_numerical01(A1, B2, C3, R, P, Y)

clc;

% 参考文献：《基于牛顿一拉夫逊迭代法的6自由度机器人逆解算法术》，作者：江南大学，王宪。

N = 100 ; % 迭代次数
T = cell(1, N+5) ; % 当前姿态矩阵
J = cell(1, N+5) ; % 当前角度的Jacobian矩阵
D = cell(1, N+5) ; % 当前的Δ
w = cell(1, N+5) ; % 当前的微分运动向量
delta = cell(1, N+5); % 当前的dθ

o = zeros(1, N*10) ; % 角度数组初始化

L(1) = Link([0   162.5   0       pi/2],  'standard');
L(2) = Link([0   0      -425     0   ],  'standard');
L(3) = Link([0   0      -392.2   0   ],  'standard');
L(4) = Link([0   133.3   0       pi/2],  'standard');
L(5) = Link([0   99.7    0      -pi/2],  'standard');
L(6) = Link([0   99.6    0       0   ],  'standard'); % 定义DH参数

six_link = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6)]); % 获得模型

T1 = rpy2r(R, P, Y); 
T2 = [A1 B2 C3]; 
Tend = [T1,T2';[0 0 0 1]]; % 定义最终矩阵

I = eye(4) ; % 定义单位矩阵

% % 定义随机初始矩阵
% T3 = randi(300, [4, 3]) ;
% T4 = [0 0 0 1] ;
% T0 = [T3; T4] ;
% 此处应该注意的是初始矩阵T1应该赋值，还是随机产生，还是给定角度再产生？

% 赋值初始矩阵
% T0(1, 1)=Tend(1, 1)+rand(); T0(1, 2)=Tend(1, 2)+rand(); T0(1, 3)=Tend(1, 3)+rand(); T0(1, 4)=Tend(1, 4)+rand();
% T0(2, 1)=Tend(2, 1)+rand(); T0(2, 2)=Tend(2, 2)+rand(); T0(2, 3)=Tend(2, 3)+rand(); T0(2, 4)=Tend(2, 4)+rand();
% T0(3, 1)=Tend(3, 1)+rand(); T0(3, 2)=Tend(3, 2)+rand(); T0(3, 3)=Tend(3, 3)+rand(); T0(3, 4)=Tend(3, 4)+rand();
% T0(4, 1)=Tend(4, 1); T0(4, 2)=Tend(4, 2); T0(4, 3)=Tend(4, 3); T0(4, 4)=Tend(4, 4); 
% 不能说为了给初始矩阵而用到逆运动学算法！

% 初始化角度与矩阵
o(1) = -pi/2; o(2) = -pi/2; o(3) = pi/2; o(4) = -pi/2; o(5) = -pi/2 ; o(6) = 0 ;
T{1} = six_link.fkine([o(1) o(2) o(3) o(4) o(5) o(6)]).T ;

% 初始化循环次数以及误差
i = 1 ;
eps = 5e-3; % 误差

% 进入循环
for i = 2:N+1
    T{i} = T{i-1} ;
    D{i} = inv(T{i}) * Tend - I ;
    w{i}(1, 1) = D{i}(1, 4) ;
    w{i}(2, 1) = D{i}(2, 4) ;
    w{i}(3, 1) = D{i}(3, 4) ;
    w{i}(4, 1) = -D{i}(2, 3) ;
    w{i}(5, 1) = D{i}(1, 3) ;
    w{i}(6, 1) = -D{i}(1, 2) ;
    J{i} = six_link.jacob0([o(i*6-11) o(i*6-10) o(i*6-9) o(i*6-8) o(i*6-7) o(i*6-6)]) ; % 求Jacobian矩阵
    delta{i} = inv(J{i}) * w{i} ;
    if norm(delta{i}) <= eps && i < N+1
        Tres = T{i} ;
        fprintf('correct!')
        break ;
    elseif norm(delta{i}) > eps && i == N+1
        error('Not convergent!')
    else
       o(i*6-5) = o(i*6-11) + delta{i}(1, 1) ; 
       o(i*6-4) = o(i*6-10) + delta{i}(2, 1) ; 
       o(i*6-3) = o(i*6-9) + delta{i}(3, 1) ; 
       o(i*6-2) = o(i*6-8) + delta{i}(4, 1) ; 
       o(i*6-1) = o(i*6-7) + delta{i}(5, 1) ; 
       o(i*6) = o(i*6-6) + delta{i}(6, 1) ; 
       T{i} = six_link.fkine([o(i*6-5) o(i*6-4) o(i*6-3) o(i*6-2) o(i*6-1) o(i*6)]).T ; 
       fprintf('now is %d times\n', i-1)
    end
end

% fprintf(Tres is %)

end

% 测试数据 IKmatrix_manual_numerical01(-133.06, 492.15, 487.47, rad2deg(-3.141), rad2deg(-0.006), rad2deg(0.002))