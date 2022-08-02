# Inverse-Kinematics
It is an unique IK algorithm based on matlab
之前的章节从数学角度介绍了运动位姿的表示，以及正运动学的DH解法（旋量法将在B站给出）和逆运动学的基本介绍，本章节将详细的告诉大家速度运动学并附上逆运动学解析解和数值解的matlab代码与讲解。
参考文献与资料：
1.《协作机器人零力控制与碰撞检测技术研究》，作者：中国科学技术大学，陈赛旋。
2.《基于牛顿一拉夫逊迭代法的6自由度机器人逆解算法术》，作者：江南大学，王宪。（这篇文献公式有误，之后会指出）
3.《现代机器人学》Lynch & Park 机械工业出版社
Github地址：https://github.com/AdamTristan?tab=repositories

一、速度运动学
![image](https://user-images.githubusercontent.com/76904279/182283340-f2e7222c-5ee5-4eeb-8ad9-ef7056399819.png)
回到我们的老朋友，二连杆，
其正向运动学方程为：
![image](https://user-images.githubusercontent.com/76904279/182283348-03c00927-7b47-4e7b-b7e6-a270e9e36b57.png)
对他左右两边同时求导有：
![image](https://user-images.githubusercontent.com/76904279/182283358-1b427396-f893-4ade-bb01-0b1b74e6fc40.png)
整理成Jacobian形式为：
![image](https://user-images.githubusercontent.com/76904279/182283364-606adf94-f684-4dd2-8a61-26337d435aef.png)
此时可以看成J矩阵为参数，使得θ线性组合的结果，若θ线性相关，则奇异；若线性无关，则张成整个维度的空间。

二、逆运动学解析解
之前我们说到，针对特定结构的机械臂（后三臂垂直），我们可以直观的peiper法解出逆运动学解析解，但是对于复杂的、泛定的结构,我们需要常规的代数解法去完成。
已知：转换矩阵和末端矩阵，令：
T61 = simplify(inv(T01) * T3)
T16 = simplify(T12 * T23 * T34 * T45 * T56) 
比较二者形式，有：
![image](https://user-images.githubusercontent.com/76904279/182283380-8e2e832e-2bbb-4845-b915-4c3df402c0f7.png)
![image](https://user-images.githubusercontent.com/76904279/182283381-3e11924a-1436-43f0-8627-a2f3703547f9.png)
比较各项位置，得出各个关节角：
![image](https://user-images.githubusercontent.com/76904279/182283394-ff8611ed-17b3-4ed7-9e0d-e31ecd0e5383.png)
![image](https://user-images.githubusercontent.com/76904279/182283400-e1fb9081-2d0a-42af-b7d0-0deebc7a6357.png)
![image](https://user-images.githubusercontent.com/76904279/182283405-4a84fdf7-1fb9-4e9e-bfe8-283294932c7e.png)
![image](https://user-images.githubusercontent.com/76904279/182283409-7fbe217d-8726-40eb-a459-31d37a8dccca.png)
![image](https://user-images.githubusercontent.com/76904279/182283416-9543d150-6d15-496c-a001-027d9cea1f3f.png)
![image](https://user-images.githubusercontent.com/76904279/182283420-9f563fb9-4cd9-4089-afe2-0dbb55fa94be.png)
![image](https://user-images.githubusercontent.com/76904279/182283430-d9518753-8da2-4aa3-b939-d705e51a8094.png)
![image](https://user-images.githubusercontent.com/76904279/182283435-8d0d6cb7-cab0-49ce-9408-e2c26cfca79d.png)
![image](https://user-images.githubusercontent.com/76904279/182283441-19bc634b-6c75-4024-b1b2-e69fd5bd4b33.png)
该方法好处是运算快，坏处是存在多解。
![image](https://user-images.githubusercontent.com/76904279/182283464-24702aa7-a576-4ca6-902e-e3f497acb937.png)
三、逆运动学数值解
首先，纠正参考文献（2）式（6）的公式错误，最终矩阵第二行第三个数字z应改为x。
逆运动学的解析解有几何法和代数法两种解法，但是数值解法则是基于Newton-Ralfsnn法，下面我将介绍数值解法的原理。
对于运动旋量有：
![image](https://user-images.githubusercontent.com/76904279/182283488-0460c802-5478-4edb-bf9e-8bb649cca4a3.png)
即：
![image](https://user-images.githubusercontent.com/76904279/182283495-63f95cb5-9530-4c73-890c-67cdc7d258f7.png)
取伪逆（对于六自由度的机械臂，该矩阵为方阵，可直接求逆）有：
![image](https://user-images.githubusercontent.com/76904279/182283499-e168acb9-17db-45e0-b13d-7b9a74a4601a.png)
对该变量Δ来说，有：
![image](https://user-images.githubusercontent.com/76904279/182283520-3c88a2aa-4f27-405c-a070-ac90be97074d.png)
综上，该数值解法的步骤为：
1.根据当前关节角度求得末端位姿
2.利用上述公式算出dθ
3.设定迭代次数N与最小误差eps，若同时满足该状态迭代次数小于N，以及小于最小误差，则退出循环，输出当前角度；否则继续循环，θ自增dθ。
该方法的好处是不存在多解，而是慢慢逼近最终解；坏处是计算收敛慢。
