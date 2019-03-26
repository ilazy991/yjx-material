
The project aimed to propose a strategy to solve the problems of obstacle avoidance and replanning for a mobile robot especially the Novus Stacker. Chapter 1 starts with a brief description of the problem of local planning. It further introduces the available local planners and the inherent problems in each of the planner. Chapter 2 discusses about Timed Elastic Bands and its ROS Package. The chapter ends with a discussion about the changes made in the algorithm for adding it to the robot navigation code. Chapter 3 explains the costmap converter plugin used along with teb local planner for providing obstacles information in the form of polygons. Chapter 4 discusses about the results obtained and the future work possible.  
该项目旨在提出一种解决移动机器人特别是Novus堆垛机避障和重新规划问题的策略。第一章首先简要介绍了地方规划问题。它进一步介绍了可用的本地规划者和每个规划者的内在问题。第二章讨论了定时弹性带及其ROS包。本章最后讨论了将该算法添加到机器人导航中的变化


Motion Planning is concerned with the development of feasible paths that respect the kinematic and dynamic motion constraints of a robot. Often this  problem in mobile robots is divided into different levels comprising of global  planning, local planning and control. While the global planner may provide  completely feasible paths to follow, however it is difficult to recompute them  everytime an obstacle crosses. Thus it becomes the responsibility of the local  planner to continuously compute smooth paths.  
运动规划是关于发展可行路径，尊重机器人的运动学和动态运动约束。移动机器人的这一问题通常分为全局规划、局部规划和控制三个层次。虽然全球计划可能提供完全可行的路径，但很难重新计算他们每次跨越障碍。因此，不断地计算平滑路径就成为了本地规划人员的责任。


The work involved research and implementation of a local planning algorithm  for the Novus Stacker. A literature survey of various available planners was  carried out and Timed Elastic Bands was chosen for implementation. Further  testing and tuning of the algorithm was carried out on MR100.  
研究和实现了一种新型堆垛机的局部规划算法。采用文献调查的方法对现有的各种规划师进行了研究，并选择了定时橡皮筋进行实施。在MR100上对算法进行了进一步的测试和优化。



Desired Characteristics  
The following characteristics are desired from a local planner: -
• Avoids Dynamic Obstacles  
• Generate smooth paths while obeying kinodynamic constraints
• Have a fast computation time for online replanning  
所需的特点  
以下特点是需要从当地规划师:-  
•避免动态障碍  
•在遵守动态约束的同时生成平滑的路径  
•具有快速的计算时间在线重新规划  



**Literature Survey  
文献调查**



Local planning is a type of motion planning that requires the robot to move in dynamic environment where its algorithm will respond to the obstacle and the change of environment. Local path planning also can be defined as real time obstacle avoidance by using sensory based information regarding contingency measures that affect the safe navigation of the robot.  
局部规划是一种要求机器人在动态环境中运动的运动规划，其算法会对障碍物和环境的变化做出响应。局部路径规划也可以定义为实时避障，利用基于感知的信息对影响机器人安全导航的应急措施进行研究。




Local plans can pe presented in two ways, namely paths or trajectories. Although these terms are often used as synonyms, there is a difference between the two. Path is expressed as a continuous sequece of configurations beginning and ending with the boundary configurations, i.e. the initial configuration and the terminating configuration respectively. In other words, a path is a geometric trace that the vehicle should follow in order to reach its destination without colliding with obstacles. On the other hand, trajectory is represented as a sequenceof states visited by the vehicle, parameterised by time and, possibly, velocity.Trajectory planning (also known as trajectory generation) is concerned withthe real-time planning of the actual robots transition from one feasible state tothe next, satisfying the robotss kinematic and dynamic limits and maintainingsmoothness while at the same time, avoiding obstacles including other robots 3 The HiTech Robotic Systemz Ltd.and humans [1]  
局部规划可以用两种方式表示，即路径或轨迹。虽然这些术语经常用作同义词，但它们之间是有区别的。路径表示为以边界构型开始和结束的构型的连续序列，即初始构型和终止构型。换句话说，路径是车辆应该遵循的几何轨迹，以便在不与障碍物碰撞的情况下到达目的地。另一方面，轨迹表示为飞行器访问的状态序列，由时间(可能还有速度)参数化。轨迹规划(又称轨迹生成)是指实际机器人从一个可行状态过渡到下一个可行状态的实时规划，既满足机器人的运动学和动力学极限，又保持平滑，同时避免包括其他机器人在内的障碍物



Most of the approaches in local motion planning deal with modification of already available global paths. It is a preferable approach due to the inherent uncertainty of the dynamic environments. One of the earliest solution which modifies a path locally is the elastic band algorithm. The main idea of the elastic band approach is to deform an originally given path by considering it as an elastic rubber band subject to internal and external forces which balance each other in the attempt to contract the path while keeping a distance from obstacles [2]. While the approach is able to handle non-holonomic kinematics as well as dyanmic obstacles, however it does not take the dynamics of the robot into account.  
局部运动规划中的大多数方法都是对已有的全局路径进行修改。由于动态环境固有的不确定性，该方法是一种较好的方法。弹性带算法是最早的局部修正路径的算法之一。弹性带法的主要思想是将原给定的路径变形为一条受内外力平衡作用的弹性橡皮筋，在与障碍物[2]保持一定距离的情况下收缩路径。该方法能够处理非完整运动学和动力学障碍，但没有考虑机器人的动力学特性。



The timed elastic band algorithm modifies the elastic band approach by incorporating temporal information thus allowing control of velocity, acceleration and jerk of the trajectory. The algorithm discretizes the initial plan into closely spaced robot configuration states which are distributed at approximately the same time gap. The problem is then solved as a scalarized multi-objective optimization problem [3] [4].  
定时弹性带算法通过结合时间信息来修改弹性带方法，从而允许控制速度和加速度轨迹的颠簸。该算法对初始规划进行了严密的离散机器人的空间构型状态近似分布于同一时间的差距。然后将问题求解为带尺度的多目标优化问题[3][4]。


Other approaches that use a discretized representation of the trajectory in configuration space include CHOMP and STOMP. Their proposed objective function contains a finite difference matrix to smooth the resulting trajectory and to additionally satisfy constraints like obstacle avoidance. CHOMP which stands for Covariant Hamiltonian Optimisation for Motion Planning, does not require that the input path to be collision free and therefore can work independantly of a global planner. Using a covarient gradient descent update rule leads to quick convergence of its trajectory to local minima [5]. Hamiltonian Monte Carlo has been suggested as a way to introduce stochasticity into the CHOMP gradient update rule. However, the method is found difficult to work in practice as it introduces additional parameters which are required to be tuned. Further it requires multiple random restarts before a successful solution is obtained [6].  
其他在构型空间中使用轨迹离散表示的方法包括CHOMP和STOMP。他们提出的目标函数包含一个有限差分矩阵来平滑最终的轨迹，并额外满足像避障这样的约束条件。CHOMP代表运动规划的协变哈密顿优化，不要求输入路径是无碰撞的，因此可以独立于全局规划器工作。利用协变梯度下降更新规则，使其轨迹快速收敛到局部极小值[5]。H

STOMP(Stochastic Trajectory Optimization for Motion Planning) uses a similar cost function to CHOMP, but in contrast it can handle cost functions for which gradients are unavailable [6]. STOMP is an algorithm that preforms local optimization, hence its performance depends upon the initial trajectory used. It cannot be expected that it will solve typical motion planning problems like alpha puzzle in reasonable amount of time. Complete comparison between TEB and STOMP is given in [4].  
译文：STOMP(用于运动规划的随机轨迹优化)使用与CHOMP类似的代价函数，但与之相反，它可以处理梯度不可用的代价函数[6]。STOMP是一种预先形成局部优化的算法，因此它的性能取决于使用的初始轨迹。不能指望它能在合理的时间内解决alpha益智等典型的运动规划问题。在[4]中给出了TEB和STOMP的完整比较。

### Timed Elastic Bands
**Introduction**


Timed Elastic Bands comes from the family of trajectory generators that explicitly augments elastic band with temporal information, thus allowing the consideration of the robots dynamic constraints such as limited robot velocities, accelerations and jerks and therefore allowing direct modification of trajectories rather than paths [3].  
译文：定时橡皮筋来自于轨迹生成器家族，它利用时间信息显式地增强橡皮筋，从而允许考虑机器人的动态约束，如有限的机器人速度、加速度和抖动，因此允许直接修改轨迹而不是路径[3]。

The ”timed elastic band” problem is formulated in a weighted multi-objective optimization framework. Most objectives are local as they depend on a few neighboring intermediate configurations. This results in a sparse system matrix for which efficient large-scale constrained least squares optimization methods exist [4].  
译文：在加权多目标优化框架下，提出了“时间弹性带”问题。大多数目标都是局部的，因为它们依赖于一些邻近的中间配置。这导致一个稀疏系统矩阵，其中有效的大规模约束最小二乘优化方法存在[4]。


The algorithm generates an initial path (using Probabilistic RoadMaps) composed of a sequence of way points into a trajectory with explicit dependence on time which enables the control of the robot in real time. Due to its modular formulation the approach is easily extended to incorporate additional objectives and constraints [7]
译文：该算法生成一个初始路径(使用概率路线图)，该路径由一系列路径点组成，并与时间显式相关，从而实现对机器人的实时控制。由于其模块化的形式，该方法很容易扩展，以纳入额外的目标和约束[7]

Objective Fuctions and Constraints  
译文：客观功能和约束


Timed Elastic Band approaches the planning problem using a hyper-graph based nonlinear optimization technique and the implementation with an open-source C++ framework called general (hyper-)graph optimization (g2o) [8] which solves graph based nonlinear optimization problems. The objective functions of the TEB belong to two types, (a) constraints and (b) objectives.

Constraints are: -  
• Penalties for Velocity constraints along x, y and theta  
• Penalties for Acceleration constraints along x, y and theta   
• Penalties for non-holonomic constraints The HiTech Robotic Systemz Ltd.  
• Penalties for Jerk along x and theta was added as an extra   constraints to make the path smoother for the robot to follow.  
• Penalties for taking the reverse direction(in differential drive robots)  


Objectives are: -   
• Distance from the obstacles  
• Distance from dynamic obstacles (dynamic obstacles are not supported presently)  
• Distance from the waypoints   
• Minimum time between two poses

Different behaviours can be obtained from the algorithm by changing the weights of the objective function. Some tuning factors are further explained in the report.

Improvement




While the original teb algorithm makes trajectories that avoid obstacles as well as follow kinodynamic constraints, however these trajectories are difficult for the robot to track accurately due to presence of jerk in the motion. Thus to make the trajectories smoother, jerk(rate of change of acceleration) was added as a constraint in the obejctive function.  
Timed Elastic Bands has been implemented as a ROS plugin (named teb local planner [9]) to the base local planner of the navigation stack. To make the code excutable in the robot navigation package, a new class named teb local planner THRSL is developed which has four public functions to create a trajectory. The functions are as follows: -  
teb local planner solves the problem of obstacle avoidance by treating them as geometric objects. When the planner is fed with a costmap 2d object, each occupied costmap cell is treated as single point-obstacle. This leads to a large amount of obstacles if the resolution of the map is high and may introduce longer computation times or instabilities in the calculation of distinctive topologies (which depend on the number of obstacles). To solve this problem teb local planner
supports the costmap converter [10] plugins. Those plugins convert occupied
costmap 2d cells to geometric primitives (poi

![image](https://github.com/ZRZheng/Jerk-Optimization-of-Timed-Elastic-Band-Algorithm/raw/master/HyperGraph.PNG)