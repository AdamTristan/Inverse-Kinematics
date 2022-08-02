clc;
syms o1 o2 o3 o4 o5 o6 R11 R12 R13 R21 R22 R23 R31 R32 R33 P1 P2 P3 d1 a2 a3 d4 d5 d6

T01 = [ cos(o1)   0        sin(o1)  0       ;
        sin(o1)   0       -cos(o1)  0       ;
        0         1        0        d1      ;
        0         0        0        1     ] ;
    
T12 = [ cos(o2)  -sin(o2)  0        a2      ;
        sin(o2)   cos(o2)  0        0       ;
        0         0        1        0       ;
        0         0        0        1     ] ;
    
T23 = [ cos(o3)  -sin(o3)  0        a3      ;
        sin(o3)   cos(o3)  0        0       ;
        0         0        1        0       ;
        0         0        0        1     ] ;

T34 = [ cos(o4)   0        sin(o4)  0       ;
        sin(o4)   0       -cos(o4)  0       ;
        0         1        0        d4      ;
        0         0        0        1     ] ;

T45 = [ cos(o5)   0       -sin(o5)  0       ;
        sin(o5)   0        cos(o5)  0       ;
        0        -1        0        d5      ;
        0         0        0        1     ] ;

T56 = [ cos(o6)  -sin(o6)  0        0       ;
        sin(o6)   cos(o6)  0        0       ;
        0         0        1        d6      ;
        0         0        0        1     ] ;
    
T61 = simplify(inv(T01) * T3)
T16 = simplify(T12 * T23 * T34 * T45 * T56) 


