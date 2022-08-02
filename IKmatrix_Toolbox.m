% 逆运动学

% 1.逆运动学解析解
T1 = six_link.fkine([pi/2 pi/2 0 pi/2 pi/2 -pi/2])
% qi = six_link.ikine6s(T1)
% T2 = six_link.fkine(qi)
% 注意到ur5e前端不满足球形关节，故无法使用解析解

% 2.逆运动学数值解
% T1 = six_link.fkine([pi/4 pi/4 0 pi/4 pi/4 -pi/4])
% 注意到不要出现90°， 否则函数无法收敛
% qi = six_link.ikine(T1)
% T2 = six_link.fkine(qi)
% angel1 = tr2rpy(T2)
% p1 = transl(T2)

% 速度运动学
T3 = six_link.jacob0([pi/4 pi/4 0 pi/4 pi/4 -pi/4]) % 0 for frame b
% T4 = six_link.jacobn([pi/4 pi/4 0 pi/4 pi/4 -pi/4]) ; % n for frame s
