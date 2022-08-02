function T = IKmatrix_manual02(A1, B2, C3, R, P, Y)

clc ;

syms o1 o12 o2 o22 o23 o232 o246 o226 o236 o2326 o234 o3 o32 o33 o34 o36 o326 o336 o346 o4 o42 o43 o44 o46 o426 o436 o446 o5 o52 o6 o62 o63 o64 o234 o2342 o2343 o2344 o23 o2322 o233 o234 o236 o23226 o2336 o2346 nx ox ax px ny oy ay wy nz oz az pz d1 a2 a3 d4 d5 d6 theta1 theta12 theta2 theta22 theta232 theta24 theta26 theta226 theta2326 theta246 theta3 theta32 theta33 theta34 theta4 theta42 theta43 theta44 theta46 theta426 theta436 theta446 theta5 theta52 theta6 theta62 theta63 theta64 theta234 theta2342 theta2343 theta2344 theta23 theta2322 theta233 theta234 theta236 theta23226 theta2336 theta2346 AA AA2 AA3 AA4 BB BB2 BB3 BB4

tic

%  获得最终矩阵
T1 = rpy2r(R, P, Y); 
T2 = [A1 B2 C3]; 
T3 = [T1,T2';[0 0 0 1]]; 

% 获得初始矩阵
T01 = [ cos(o1)   0        sin(o1)  0       ;
        sin(o1)   0       -cos(o1)  0       ;
        0         1        0        162.5   ;
        0         0        0        1     ] ;
    
T12 = [ cos(o2)  -sin(o2)  0       -425     ;
        sin(o2)   cos(o2)  0        0       ;
        0         0        1        0       ;
        0         0        0        1     ] ;
    
T23 = [ cos(o3)  -sin(o3)  0       -392.2   ;
        sin(o3)   cos(o3)  0        0       ;
        0         0        1        0       ;
        0         0        0        1     ] ;

T34 = [ cos(o4)   0        sin(o4)  0       ;
        sin(o4)   0       -cos(o4)  0       ;
        0         1        0        133.3   ;
        0         0        0        1     ] ;

T45 = [ cos(o5)   0       -sin(o5)  0       ;
        sin(o5)   0        cos(o5)  0       ;
        0        -1        0        99.7    ;
        0         0        0        1     ] ;

T56 = [ cos(o6)  -sin(o6)  0        0       ;
        sin(o6)   cos(o6)  0        0       ;
        0         0        1        99.6    ;
        0         0        0        1     ] ;
    
T61 = simplify(inv(T01) * T3) ;
T16 = simplify(T12 * T23 * T34 * T45 * T56) ;
% 事实上，此处求解不需要以上矩阵，我仅仅是将矩阵的DH参数列出，同时告诉大家验证T16与T61一一对应从而推出以下公式

% 求解theta1

% 直接求解法
% eqn1 = T61(3, 3) == T61(3, 3)
% eqn2 = T61(3, 4) == T61(3, 4)
% o1 = solve(eqn1, eqn2, o1)
% 尝试直接求解，发现不行，故人工化简求解：根据博士论文文献《协作机器人零力控制与碰撞检测技术研究》作者：陈赛旋 学校：中国科学技术大学

% 化简求解法 
% T16 = [ nx  ox  ax  px   ; == T3
%         ny  oy  ay  wy   ;
%         nz  oz  az  pz   ;
%         0   0   0   1  ] ;

% 赋值
nx = T3(1, 1) ; ox = T3(1, 2) ; ax = T3(1, 3) ; px = T3(1, 4) ;
ny = T3(2, 1) ; oy = T3(2, 2) ; ay = T3(2, 3) ; wy = T3(2, 4) ;
nz = T3(3, 1) ; oz = T3(3, 2) ; az = T3(3, 3) ; pz = T3(3, 4) ;
d1 = 162.5 ; a2 = -425 ; a3 = -392.2 ; d4 = 133.3 ; d5 = 99.7 ; d6 = 99.6 ;

% 计算theta1
o1 = atan2(d4, sqrt((d6*ay-wy))^2+(px-d6*ax)^2-(d4)^2)-atan2(d6*ay-wy, px-d6*ax) ;
o12 = atan2(d4, -sqrt((d6*ay-wy))^2+(px-d6*ax)^2-(d4)^2)-atan2(d6*ay-wy, px-d6*ax) ;
theta1 = vpa(rad2deg(o1), 3) 
theta12 = vpa(rad2deg(o12), 3) 

% 计算theta5
o5 = atan2(sqrt((nx*sin(o1)-ny*cos(o1))^2+(ox*sin(o1)-oy*cos(o1))^2), ax*sin(o1)-ay*cos(o1)) ;
o52 = atan2(sqrt((nx*sin(o12)-ny*cos(o12))^2+(ox*sin(o12)-oy*cos(o12))^2), ax*sin(o12)-ay*cos(o12)) ;
theta5 = vpa(rad2deg(o5), 3) 
theta52 = vpa(rad2deg(o52), 3) 

% 计算theta6
if abs(o5-0) < 0.5 || abs(o5-180) < 0.5
    error('奇异！无法求得θ6！')
else
    o6 = atan2(-(ox*sin(o1)-oy*cos(o1))/sin(o5), (nx*sin(o1)-ny*cos(o1))/sin(o5)) ;
    o62 = atan2(-(ox*sin(o12)-oy*cos(o12))/sin(o5), (nx*sin(o12)-ny*cos(o12))/sin(o5)) ;
    o63 = atan2(-(ox*sin(o12)-oy*cos(o12))/sin(o52), (nx*sin(o12)-ny*cos(o12))/sin(o52)) ;
    o64 = atan2(-(ox*sin(o1)-oy*cos(o1))/sin(o52), (nx*sin(o1)-ny*cos(o1))/sin(o52)) ;
    theta6 = vpa(o6, 3)
    theta62 = vpa(o62, 3)
    theta63 = vpa(o63, 3) 
    theta64 = vpa(o64, 3) 
end

% 计算theta234
if abs(o5-0) < 0.5 || abs(o5-180) < 0.5
    error('奇异！无法求得θ2！')
else
%     o234 = atan2(-az/sin(o5), -(ax*cos(o1)+ay*sin(o1))/sin(o5)) ;
    o234 = atan2(az, ax*cos(o1)+ay*sin(o1)) ;
    o2342 = atan2(az, ax*cos(o12)+ay*sin(o12)) ;
    o2343 = atan2(az, ax*cos(o12)+ay*sin(o12)) ;
    o2344 = atan2(az, ax*cos(o1)+ay*sin(o1)) ;
    theta234 = vpa(o234, 3) ;
    theta2342 = vpa(o2342, 3) ;
    theta2343 = vpa(o2343, 3) ;
    theta2344 = vpa(o2344, 3) ;
end

% 计算theta2
AA = px*cos(o1)+wy*sin(o1)-d5*sin(o234)+d6*sin(o5)*cos(o234) ; 
AA2 = px*cos(o12)+wy*sin(o12)-d5*sin(o2342)+d6*sin(o5)*cos(o2342) ; 
AA3 = px*cos(o12)+wy*sin(o12)-d5*sin(o2343)+d6*sin(o52)*cos(o2343) ; 
AA4 = px*cos(o1)+wy*sin(o1)-d5*sin(o2344)+d6*sin(o52)*cos(o2344) ; 
BB = pz-d1+d5*cos(o234)+d6*sin(o5)*sin(o234) ;
BB2 = pz-d1+d5*cos(o2342)+d6*sin(o5)*sin(o2342) ;
BB3 = pz-d1+d5*cos(o2343)+d6*sin(o52)*sin(o2343) ;
BB4 = pz-d1+d5*cos(o2344)+d6*sin(o52)*sin(o2344) ;
o2 = atan2(AA^2+BB^2+a2^2-a3^2, sqrt(4*a2^2*(AA^2+BB^2)-(AA^2+BB^2+a2^2-a3^2)^2))-atan2(AA, BB) ;
o22 = atan2(AA2^2+BB2^2+a2^2-a3^2, sqrt(4*a2^2*(AA2^2+BB2^2)-(AA2^2+BB2^2+a2^2-a3^2)^2))-atan2(AA2, BB2) ;
o232 = atan2(AA3^2+BB3^2+a2^2-a3^2, sqrt(4*a2^2*(AA3^2+BB3^2)-(AA3^2+BB3^2+a2^2-a3^2)^2))-atan2(AA3, BB3) ;
o24 = atan2(AA4^2+BB4^2+a2^2-a3^2, sqrt(4*a2^2*(AA4^2+BB4^2)-(AA4^2+BB4^2+a2^2-a3^2)^2))-atan2(AA4, BB4) ;
o26 = atan2(AA^2+BB^2+a2^2-a3^2, -sqrt(4*a2^2*(AA^2+BB^2)-(AA^2+BB^2+a2^2-a3^2)^2))-atan2(AA, BB) ;
o226 = atan2(AA2^2+BB2^2+a2^2-a3^2, -sqrt(4*a2^2*(AA2^2+BB2^2)-(AA2^2+BB2^2+a2^2-a3^2)^2))-atan2(AA2, BB2) ;
o2326 = atan2(AA3^2+BB3^2+a2^2-a3^2, -sqrt(4*a2^2*(AA3^2+BB3^2)-(AA3^2+BB3^2+a2^2-a3^2)^2))-atan2(AA3, BB3) ;
o246 = atan2(AA4^2+BB4^2+a2^2-a3^2, -sqrt(4*a2^2*(AA4^2+BB4^2)-(AA4^2+BB4^2+a2^2-a3^2)^2))-atan2(AA4, BB4) ;
theta2 = vpa(rad2deg(o2), 3) 
theta22 = vpa(rad2deg(o22), 3) 
theta232 = vpa(rad2deg(o232), 3) 
theta24 = vpa(rad2deg(o24), 3) 
theta26 = vpa(rad2deg(o26), 3) 
theta226 = vpa(rad2deg(o226), 3) 
theta2326 = vpa(rad2deg(o2326), 3) 
theta246 = vpa(rad2deg(o246), 3) 

% 计算theta23
o23 = atan2(pz-d1+d5*cos(o234)-a2*sin(o2)+d6*sin(o5)*sin(o234), px*cos(o1)+wy*sin(o1)-d5*sin(o234)-a2*cos(o2)+d6*sin(o5)*cos(o234)) ;
o2322 = atan2(pz-d1+d5*cos(o2342)-a2*sin(o22)+d6*sin(o5)*sin(o2342), px*cos(o12)+wy*sin(o12)-d5*sin(o2342)-a2*cos(o22)+d6*sin(o5)*cos(o2342)) ;
o233 = atan2(pz-d1+d5*cos(o2343)-a2*sin(o232)+d6*sin(o52)*sin(o2343), px*cos(o12)+wy*sin(o12)-d5*sin(o2343)-a2*cos(o232)+d6*sin(o52)*cos(o2343)) ;
o234 = atan2(pz-d1+d5*cos(o2344)-a2*sin(o24)+d6*sin(o52)*sin(o2344), px*cos(o1)+wy*sin(o1)-d5*sin(o2344)-a2*cos(o24)+d6*sin(o52)*cos(o2344)) ;
o236 = atan2(pz-d1+d5*cos(o234)-a2*sin(o26)+d6*sin(o5)*sin(o234), px*cos(o1)+wy*sin(o1)-d5*sin(o234)-a2*cos(o26)+d6*sin(o5)*cos(o234)) ;
o23226 = atan2(pz-d1+d5*cos(o2342)-a2*sin(o226)+d6*sin(o5)*sin(o2342), px*cos(o12)+wy*sin(o12)-d5*sin(o2342)-a2*cos(o226)+d6*sin(o5)*cos(o2342)) ;
o2336 = atan2(pz-d1+d5*cos(o2343)-a2*sin(o2326)+d6*sin(o52)*sin(o2343), px*cos(o12)+wy*sin(o12)-d5*sin(o2343)-a2*cos(o2326)+d6*sin(o52)*cos(o2343)) ;
o2346 = atan2(pz-d1+d5*cos(o2344)-a2*sin(o246)+d6*sin(o52)*sin(o2344), px*cos(o1)+wy*sin(o1)-d5*sin(o2344)-a2*cos(o246)+d6*sin(o52)*cos(o2344)) ;
theta23 = vpa(rad2deg(o23), 3) ;
theta2322 = vpa(rad2deg(o2322), 3) ;
theta233 = vpa(rad2deg(o233), 3) ;
theta234 = vpa(rad2deg(o234), 3) ;
theta236 = vpa(rad2deg(o236), 3) ;
theta23226 = vpa(rad2deg(o23226), 3) ;
theta2336 = vpa(rad2deg(o2336), 3) ;
theta2346 = vpa(rad2deg(o2346), 3) ;

% 计算theta3
o3 = o23 - o2 ;
o32 = o2322 - o22 ;
o33 = o233 - o232 ;
o34 = o234 - o24 ;
o36 = o236 - o26 ;
o326 = o23226 - o226 ;
o336 = o2336 - o2326 ;
o346 = o2346 - o246 ;
theta3 = vpa(rad2deg(o3), 3) 
theta32 = vpa(rad2deg(o32), 3) 
theta33 = vpa(rad2deg(o33), 3) 
theta34 = vpa(rad2deg(o34), 3) 
theta36 = vpa(rad2deg(o36), 3) 
theta326 = vpa(rad2deg(o326), 3) 
theta336 = vpa(rad2deg(o336), 3) 
theta346 = vpa(rad2deg(o346), 3) 

% 计算theta4
o4 = o234 - o23 ;
o42 = o2342 - o2322;
o43 = o2343 - o233 ;
o44 = o2344 - o234 ;
o46 = o234 - o236 ;
o426 = o2342 - o23226 ;
o436 = o2343 - o2336 ;
o446 = o2344 - o236 ;
theta4 = vpa(rad2deg(o4), 3)
theta42 = vpa(rad2deg(o42), 3)
theta43 = vpa(rad2deg(o43), 3)
theta44 = vpa(rad2deg(o44), 3)
theta46 = vpa(rad2deg(o46), 3)
theta426 = vpa(rad2deg(o426), 3)
theta436 = vpa(rad2deg(o436), 3)
theta446 = vpa(rad2deg(o446), 3)


toc

T = [theta1 theta2 theta3 theta4 theta5 theta6]
end

% 测试数据
% IKmatrix_manual02(-1.41, -233.21, 1080.15, -90, 0, 180)
% IKmatrix_manual02(-133.06, 492.15, 487.47, rad2deg(-3.141), rad2deg(-0.006), rad2deg(0.002))
% IKmatrix_manual02(-785.27, -385.99, 421.04, rad2deg(-0.36), rad2deg(0.375), rad2deg(-1.143))
% IKmatrix_manual02(82.17, 474.93, 385.09, rad2deg(3.128), rad2deg(-0.173), rad2deg(-1.474))

