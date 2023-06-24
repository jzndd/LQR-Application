# LQR Application

> author : jzndd

## Path Planning

+ LQRPlanner 主要是设计了 LQR 所需要的类 

+ LQRRRTStar 文章中使用的 LQR-RRT* 进行路径规划的主要代码

+ CubicSpline 这是为后面的路径跟踪服务的

  但其实本人感觉 LQR  并不适用于进行路径规划

## Fix-Value Tracking

  执行定值跟踪，LQRINIT.m 是为了对系统（包括A,B,K什么的）进行初始化，simulation.slx 是仿真文件

## Path Tracking

  执行路径跟踪，这应该是 LQR 的老本行了，但经过这几天的学习本人认为：路径跟踪内在逻辑分为两步：根据要经过的目标点进行路径规划，然后再根据规划好的路径进行跟踪，LQR 负责的往往是后者

  + lqr_steer_control 给出了路径跟踪
  + lqr_speed_steer_control 给出了速度-路径跟踪


代码参考：
https://github.com/zhm-real/MotionPlanning
https://github.com/mingyucai/LQR_CBF_rrtStar
https://zhuanlan.zhihu.com/p/504299366
https://github.com/WenchaoDing/PythonRobotics
   