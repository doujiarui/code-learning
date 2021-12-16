<<<<<<< HEAD
# ORBSLAM2_SourceCode

<img src="C:\Users\djr\AppData\Roaming\Typora\typora-user-images\image-20211206104441902.png" alt="image-20211206104441902" style="zoom:80%;" />

## 不管以哪种方式运行SLAM都会实例化==System.cc：==

1. 确定传感器类型并读取ORB字典，为后期的回环检测做准备；

2. 创建关键帧数据库KeyFrameDatabase，用于存放关键帧相关数据；

3. 初始化Tracking线程。其实Tracking线程是在main线程中运行的，可以简单的认为main线程就是Tracking线程；

4. 初始化并启动LocalMapping线程；

5. 初始化并启动LoopClosing线程；

6. 初始化并启动窗口显示线程mptViewer；

7. 在各个线程之间分配资源，方便线程彼此之间的数据交互。

   

## 以跑数据集为例

根据传感器类型传入带有不同参数的图片（左右图、深度等），调用Tracking线程名为==GrabImag+相机类型==的函数， 然后返回相机的位姿（世界坐标系到该帧相机坐标系的变换矩阵）

 - TrackMonocular->GrabImagMonocular     return Mat
 - TrackRGBD->GrabImagRGBD                  return Mat
 - TrackStereo->GrabImagStereo    		return Mat

## 以单目相机为例，看代码的执行过程：

实时传入图片，==Tracking::GrabImagMonocular(img, timestamp)==

- 图像转灰度图像 
- 构造Frame（ORB特征点提取；初始化时会提取更多的特征点）
- 跟踪
  - 从上一帧进行初始化位姿、世界坐标系或者进行重定位
  - 跟踪局部地图
  - 新的关键帧生成
  - Tracking线程的输出为新生成的关键帧，并将新生成的关键帧传给LocalMapping线程继续后边的工作
- 返回当前帧的位姿-世界坐标系到该帧相机坐标系的变换矩阵

## ==Tracking构造函数：==

- 加载相机参数、构造相机内参矩阵、图像矫正系数
- 加载ORB提取特征点有关参数，并创建特征提取器
  - 不管什么类型的相机都会用到mpORBextractorLeft
  - 双目情况下创建mpORBextractorRight
  - 初始化会用到mpIniORBextractor，目的是为了提取更多的特征点能够更好的初始化地图
  - 深度和基线的参数设置
- 设置LocalMapper、LoopClosing、Viewer为了资源共享

## ==ORBextractor构造函数：==

- 确定金字塔每一层缩放的系数和$\sigma^2$
- 分配金字塔每一层特征点个数
- 关于描述子、旋转计算有关参数设置

## ==Frame构造函数：==

- 为双目相机准备的构造函数
- 为RGBD相机准备的构造函数
- 为单目相机准备的构造函数
  - 获取ORBextractor实例化的对象中设置的相关参数
  - 提取特征点、关键点、描述子，矫正去畸变
  - 分配点到网格图像中



## ==Tracking::Track()：==

利用状态机实现状态跳转

- ### 初始化

  - 对前两帧的图像初始化和特征点匹配（要求匹配的特征点数>100）
  - 完成初始化后再有图像传入就是做位姿估计
- ### 跟踪局部地图
- 记录位姿，用于轨迹复现





## LoopClosing线程工作内容

- 闭环检测，获得闭环候选帧集合；
- 几何验证，确定闭环帧，计算sim3,根据sim3的计算值更新地图点的位姿；
- 进行地图点融合和位姿优化；

以millisecond(5)的时间间隔执行主线程，其代码非常简洁：

```c++
if CheckNewKeyFrmes()
	if DetectLoop()
		if ComputeSim3()
			CorrectLoop()
```

```c++
bool LoopClosing::DetectLoop()
{
    /*
    @brief
    
    Step1:
	从队列取出最早进来的关键帧，并且回环间隔大于10帧
	
	Step2:
	取出所有当前帧的共视关键帧（>15个共视地图点）,计算关键帧的最小得分minScore
	
	Step3:
	KeyFrameDatabase::DetectLoopCandidates(pKF,minScore)筛选候选集
	
	Step4:
	连续性检测进一步筛选
		- 取候选帧构建候选帧组
			-遍历之前的候选帧组
				- 如果当前候选帧组中的帧存在于之前的候选帧组，我们成为连续，记录连续数量
				- 如连续数量超过阈值3，我们将这个关键帧放入候选帧集合
	
	*/
}
```

```c++
vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(pKF,minScore)
{
    /*
    
    @brief
    - 在关键帧数据库中根据公共的词汇和相似度阈值minScore筛选候选帧集合
    - 分组计算阈值minScoreToRetain，属于一个组的关键帧就只剩一个，进一步缩小候选集
    - 0.75*minScoreToRetain再筛选
    
    Step1:
    计算关键帧数据库中的关键帧的与当前帧的最大共视词汇数量，选取候选帧的第一步
        - 取出当前帧的局部相连的关键帧剔除，不参与回环
        - 遍历当前帧的词汇
        	- 遍历包含该词汇的关键帧
        		- 记录最大word个数，把有公共单词的关键帧push到SharingWords（候选回环帧）
	
	Step2:
	确定最小公共词汇数为最大公共词汇数的0.8倍
	
	Step3:
	遍历上述闭环候选帧SharingWords
		- 用mBowVec计算相似度得分
		- 挑出 词汇数量>最小词汇数 && 得分>minScore 的关键帧
		- 组成lScoreAndMatch候选帧集合
	
	@brief
	单独计算当前帧和某一个关键帧的相似性是不够的，这里将与关键帧相连（权值最高、共视程度最高）的前十个关键帧归为一组
	Step4:
	- 把候选帧的前十个共视帧组成一组
	- 计算每一组的总得分以及得分最高的帧和分
	- 计算这几组的最高分bestAccScore
	- 计算阈值minScoreToRetain = 0.75*bestAccScore
	
	Step5:
	- 取组中得分大于minScoreToRetain的候选关键帧集合，返回
	
	
	*/
}
```

```C
bool LoopClosing::ComputeSim3()
{
    /*
    @brief
    - 根据特征点数筛选候选帧集合，添加Sim3Solver求解器
    - Sim3求解，根据 内点数量&&是否收敛 进一步筛选候选帧集合，找出闭环帧
    - 检查闭环帧的共视关键帧和他们的地图点，根据Sim3变换，计算总的特征点的数量，根据阈值判断是不是闭环成功
    
    Step1:
    遍历候选帧集，通过词袋加速算法选出匹配的特征点数大于阈值20的候选帧集合，并为每一个候选帧构造一个Sim3Solver
    
    Step2：
    - 使用RANSAC法，对每一个候选帧求解候选帧到当前帧的sim3的变换
    - 如果没有收敛则剔除
    - 如果算出Sim3变换，求解更多的特征点匹配，因为SearchByBow匹配可能会有遗漏
    	-查找更多的地图点匹配，双向优化的方法
    	-优化Sim3，然后确定内点数量
    - 内点数大于20，找到闭环帧，停止迭代，算出世界坐标系w到候选帧m的sim3变换，w->c的sim3变换 
    
    Step3:
    取出闭环帧mpMatchedKF的共视关键帧vpLoopConnectedKFs，形成闭环帧小组，把这些共视关键帧的的地图点，放入mvpLoopMapPoints
    
    Step4:
    将闭环关键帧及其连接关键帧的所有地图点投影到当前关键帧进行投影匹配，根据sim变换，查找更多的匹配，成功的闭环匹配需要满足足够多的匹配特征点数
    
    Step5:
    统计当前帧和闭环帧（包括共视）匹配地图点数目，如果超过阈值40说明成功闭环，否则失败
    
    */
}
```

# sim3求解


单目SLAM在运行过程中，不但会积累旋转和平移上的误差，还会产生尺度上的漂移。ORB-SLAM2中，对于单目的尺度漂移，回环优化的时候会做特殊的处理，即sim3的优化。

基本问题为一些已知的点是在两个不同的坐标下的测量，目的是找到这两个坐标系的变换，和ICP不同的是，还需要多求一个尺度量。

<img src="C:\Users\djr\AppData\Roaming\Typora\typora-user-images\image-20211211114035238.png" alt="image-20211211114035238" style="zoom:50%;" />



## Definition:

两个坐标系的坐标分别为$r_{l,1}, r_{l,2}, r_{l,3}$和$r_{r,1}, r_{r,2}, r_{r,3}$，现在只讨论左坐标系，

- X轴：$x^e=x_l/||x_l||, x_{l}=r_{l,2}-r_{l,1}$
	
- Y轴：$y^e=y_l/||y_l||, y_l=(r_{l,3}-r_{l,1})-[(r_{l,3}-r_{l,1})*x_{l}^e]x_l^e$
	
- Z轴：$z^e=x^e×y^e$

得到左右坐标系的基底：

- $M_l=[x^e_l,y^e_l,z^e_l],M_r=[x^e_r,y^e_r,z^e_r]$

假设左边坐标系有一向量$r_l$，那么变换到右坐标系为:

- $r_r=M_rM_l^Tr_l$
- $R=M_rM_l^T$

假设有n个点，我们可以找到：

- $r_r=sR(r_l)+r_0,||R(r_l)||^2=||r_l||^2,r_0是平移量$，

构造误差模型：

- $e_i=r_{r,i}-sR(r_{l,i})-r_0$

最小误差平方和模型：


- $\Sigma_{i=1}^n||e_i||^2$

  

## 计算平移量

计算的左右坐标系中所有点的质心：

- $\hat{r_l}=\frac{1}{n}\sum\limits_{i=1}^nr_{l,i}$
- $\hat{r_r}=\frac{1}{n}\sum\limits_{i=1}^nr_{r,i}$

则每一个点距离质心的距离为：

- $r_{l,i}^{\prime}=r_{l,i}-\hat{r_l}$
- $r_{r,i}^{\prime}=r_{r,i}-\hat{r_r}$
- $\sum\limits_{i=1}^{n}r^{\prime}_{l,i}=0$
- $\sum\limits_{i=1}^{n}r^{\prime}_{r,i}=0$

变换公式：

- $r^{\prime}_{r,i}=sR(r^{\prime}_{l,i})+r^{\prime}_0$

$$
\begin{align*}
r^{\prime}_0
& = r^{\prime}_{r,i}-sR(r^{\prime}_{l,i}) \\
& = r_{r,i}-\hat{r_r} - sR(r_{l,i}-\hat{r_l}) \\
& = r_{r,i}-\hat{r_r} - sR(r_{l,i})+sR(\hat{r_l}) \\
& = r_0-\hat{r_r}+sR(\hat{r_l})
\end{align*}
$$

优化函数：

- $\sum\limits_{i=1}^n||e_i||^2=\sum\limits_{i=1}^n||r_{r,i}-sR(r_{l,i})-r_0||^2$
- 算是两种优化函数，注意不是等价过来的
- $\sum\limits_{i=1}^n||e_i||^2=\sum\limits_{i=1}^n||r_{r,i}^{\prime}-[sR(r_{l,i}^{\prime})+r_0^{\prime}]||^2$
- $\sum\limits_{i=1}^n||e_i||^2=\sum\limits_{i=1}^n||r_{r,i}^{\prime}-sR(r_{l,i}^{\prime})||^2+n||r_0^{\prime}||^2,2r_0^{\prime}\sum\limits_{i=1}^n[r_{r,i}^{\prime}-sR(r_{l,i}^{\prime})]=0$

注意到我们的优化函数为非负，且只有$r_0^{\prime}=0$的情况下取得最小值，带入下面的式子：

$$
\begin{align*}
r^{\prime}_0
& = r^{\prime}_{r,i}-sR(r^{\prime}_{l,i}) \\
& = r_{r,i}-\hat{r_r} - sR(r_{l,i}-\hat{r_l}) \\
& = r_{r,i}-\hat{r_r} - sR(r_{l,i})+sR(\hat{r_l}) \\
& = r_0-\hat{r_r}+sR(\hat{r_l})
\end{align*}
$$
有平移量$t=r_0=\hat{r_r}-sR(\hat{r_l})$

## 尺度计算，两种情况的表达形式

由于$r_0^{\prime}=0$,所以可得误差函数：

$$
\begin{align*}
\sum\limits_{i=1}^n||e_i||^2
& =\sum\limits_{i=1}^n||r_{r,i}^{\prime}-[sR(r_{l,i}^{\prime}+r_0^{\prime})]||^2 \\
& =\sum\limits_{i=1}^n||r_{r,i}^{\prime}-sR(r_{l,i}^{\prime})||^2 \\
& = \sum\limits_{i=1}^n||r_{r,i}^{\prime}||^2-2s\sum\limits_{i=1}^nr_{r,i}^{\prime}R(r_{l,i}^{\prime})+s^2\sum\limits_{i=1}^n||R(r_{l,i}^{\prime})||^2 \\
记& = S_r-2sD+s^2S_l \\
& =(s\sqrt{S_l}-D/\sqrt{S_l})^2+(S_rS_l-D^2)/S_l
\end{align*}
$$
要是误差函数最小值，第一项必须为0，此时：

- $s=D/S_l=[\sum\limits_{i=1}^nr_{r,i}^{\prime}R(r_{l,i}^{\prime})]/\sum\limits_{i=1}^n||R(r_{l,i}^{\prime})||^2$

如果两组测量中的误差相似，则对误差项使用对称表达式更为合理：

- 原式$e_i=r_{r,i}^{\prime}-sR(r_{l,i}^{\prime})-r_0^{\prime}=r_{r,i}^{\prime}-sR(r_{l,i}^{\prime})$
- 做根号s的变换为$e_i=\frac{1}{\sqrt{S}}r_{r,i}^{\prime}-\sqrt{s}R(r_{l,i}^{\prime})$
- $\sum\limits_{i=1}^n||e_i||^2=(\sqrt{s}S_l-\frac{1}{\sqrt{s}}S_r)^2+2(S_lS_r-D)$

求改式最小值，得：

- $s=(\frac{\sum\limits_{i=1}^n||r_{r,i}^{\prime}||^2}{\sum\limits_{i=1}^n||r_{l,i}^{\prime}||^2})^\frac{1}{2}$

从这个表达式我们可以看出，他的结果不依赖于旋转，也就是说允许人们在不知道旋转的情况下确定尺度。



## 计算R

- 为了求解最小误差项，我们应该去求最大的$D=\sum\limits_{i=1}^nr_{r,i}^{\prime}R(r_{l,i}^{\prime})$
- 引入四元数来表达

$$
\begin{align*}
D&=\sum\limits_{i=1}^nr_{r,i}^{\prime} \cdot R(r_{l,i}^{\prime}),我们要清楚这里是内积，满足交换律\\
&=\sum\limits_{i=1}^n R(r_{l,i}^{\prime}) \cdot r_{r,i}^{\prime}\\
&=\sum\limits_{i=1}^n q\mathring{r_{l,i}^{\prime}}q^* \cdot \mathring{r_{r,i}^{\prime}}\\
&=\sum\limits_{i=1}^n q\mathring{r_{l,i}^{\prime}} \cdot \mathring{r_{r,i}^{\prime}}q \\
&=\sum\limits_{i=1}^n \overline\R_{l,i} q \cdot \R_{r,i}q\\
没懂&=\sum\limits_{i=1}^n q^T\overline\R_{l,i}^T  \cdot \R_{r,i}q\\
&=q^TNq\\
\end{align*}
$$

N是对称矩阵，其中的元素用坐标表示，这里省略表示形式，具体看论文

对N矩阵进行特征值分解求解最大特征值对应的特征向量，就是q，==不是很理解==

- q已知，可求欧拉角、R，然后再求平移量t和尺度s



## 闭环

将当前相机的位姿和场景与回环帧处的位姿和场景进行融合，从而消除累计误差，矫正运动轨迹。



<img src="C:\Users\djr\AppData\Roaming\Typora\typora-user-images\image-20211213140312972.png" alt="image-20211213140312972" style="zoom:67%;" />


```C++
void LoopClosing::CorrectLoop()
{
    /*
    @brief
    
	Step0:
	等局部地图结束，结束BA线程，为闭环做准备
	
	Step1:
	更新当前关键帧与其它关键帧之间的连接关系
	
	Step2:
		- mg2oScw=gScm*gSmw 闭环帧到当前帧的转换，世界坐标系，属于优化的，gScm为sim3求解优化的
		- 固定当前帧，求解闭环帧组(共视)的优化位姿和无优化位姿，指的是w->m的坐标变换
		- 遍历共视帧的地图点
			- 将该未校正的eigP3Dw先从世界坐标映射到未校正的pKFi相机坐标系，然后再反射到校正的世界坐标系下
			- 达到更新pMPi世界坐标的目的
		- 更新闭环共视帧组的信息，位置、权值等
	
	Step3:
		- 检查当前帧的地图点有无与投影匹配上的点，对匹配的的点进行替换(优化的地图点更精确)，或者填补，因为现有的地图点很可能存在累计误差
		- 疑问：闭环投影的点有没有可能在当前帧不存在
		
	Step4:
		- 将这些已纠正位姿的MapPoints与闭环MapPoints融合
		
	Step5:
		- 更新关键帧的连接关系
	
	Step6:
		- 本质图优化,Optimizer.cc文件中的优化器都是静态函数
		- OptimizeEssentialGraph()
		
	Step7:
		- 互相添加回环边
		
	Step8:
		- BA
	
	*/
}
```



## Optimizer



## 计算相机运动——前端，个人理解根据帧与帧之间的位姿估计，优化单个的位姿

- ### 单目，根据两组2D点估计运动，对极几何解决

- ### RGBD、双目，根据两组3D点估计运动，通常用ICP解决

- ### 如果一组为3D，一组为2D，该问题通过PnP（DLT、P3P、EPnP、UPnP）求解，RGBD、双目



## 对几极何

- $s_1p_1=KP,s_2p_2=K(RP+t)$
- 尺度意义下的相等,$sp\simeq p$
- $p_1\simeq KP,p_2\simeq K(RP+t)$
- $x_1=K^{-1}p_1,x_2=K^{-1}p_2$
- $x_2\simeq K^{-1}p_2 \simeq RP+t \simeq Rx_1+t$
- 左右做 $t\times$ 处理，$t\times x_2 \simeq t\times Rx_1 + 0$
-  左右乘$x^T_2$，$x^T_2t\times x_2 \simeq x^T_2t\times Rx_1$
- $x^T_2t\times Rx_1=0$
- $p^T_2x^T_2t\times Rx_1p_1=0$，记为对极约束
- 进一步简化对极约束，本质矩阵 $E=t\times R$ ，基础矩阵$F=K^{-T}EK^{-1}$，$x_2^TEx_1=p_2^TFp_1=0$
- 根据配对的像素点求$F\to E\to R,t$

### 本质矩阵E的求解,$F$也是这么求解的

- 八点法才能求解$E$
- 由奇异值的分解$SVD$恢复相机运动的$R,t$
  - $E=U\sum V^T$



## PnP(Perspective-n-Point)，描述了当知道n个3D空间点及其投影位置时，如何估计相机的位姿。

### 直接线性变换DLT，最少匹配6对点实现矩阵的线性求解

### P3P

### 最小化重投影误差求解PnP（BA）





## 后端——以全局的重投影误差来优化每一帧的位姿，五种优化方式

- ### 卡尔曼滤波器
- ### 扩展卡尔曼滤波器
- ### BA：把一路上所有的坐标点与位姿整体放在一起作为自变量进行非线性优化
- ### Pose Graph：在优化几次以后把特征点固定住不再优化，只当做位姿估计的约束，之后主要优化位姿。
- ### 因子图，因子图是增量的处理后端优化。机器人是运动的，不同的边和节点会不断加入图中，每加入一个点，普通图优化是对整个图进行优化，比较麻烦和耗时，因子图相当于保留中间结果，每加入一个点，只计算需要计算的点即可，加速计算，避免冗余计算。
- ### 在ORBSLAM2中，本质图优化，局部BA，关键帧只和回环相关，构成本质图
=======
# ORBSLAM2_SourceCode

<img src="C:\Users\djr\AppData\Roaming\Typora\typora-user-images\image-20211206104441902.png" alt="image-20211206104441902" style="zoom:80%;" />

## 不管以哪种方式运行SLAM都会实例化==System.cc：==

1. 确定传感器类型并读取ORB字典，为后期的回环检测做准备；

2. 创建关键帧数据库KeyFrameDatabase，用于存放关键帧相关数据；

3. 初始化Tracking线程。其实Tracking线程是在main线程中运行的，可以简单的认为main线程就是Tracking线程；

4. 初始化并启动LocalMapping线程；

5. 初始化并启动LoopClosing线程；

6. 初始化并启动窗口显示线程mptViewer；

7. 在各个线程之间分配资源，方便线程彼此之间的数据交互。

   

## 以跑数据集为例

根据传感器类型传入带有不同参数的图片（左右图、深度等），调用Tracking线程名为==GrabImag+相机类型==的函数， 然后返回相机的位姿（世界坐标系到该帧相机坐标系的变换矩阵）

 - TrackMonocular->GrabImagMonocular     return Mat
 - TrackRGBD->GrabImagRGBD                  return Mat
 - TrackStereo->GrabImagStereo    		return Mat

## 以单目相机为例，看代码的执行过程：

实时传入图片，==Tracking::GrabImagMonocular(img, timestamp)==

- 图像转灰度图像 
- 构造Frame（ORB特征点提取；初始化时会提取更多的特征点）
- 跟踪
  - 从上一帧进行初始化位姿、世界坐标系或者进行重定位
  - 跟踪局部地图
  - 新的关键帧生成
  - Tracking线程的输出为新生成的关键帧，并将新生成的关键帧传给LocalMapping线程继续后边的工作
- 返回当前帧的位姿-世界坐标系到该帧相机坐标系的变换矩阵

## ==Tracking构造函数：==

- 加载相机参数、构造相机内参矩阵、图像矫正系数
- 加载ORB提取特征点有关参数，并创建特征提取器
  - 不管什么类型的相机都会用到mpORBextractorLeft
  - 双目情况下创建mpORBextractorRight
  - 初始化会用到mpIniORBextractor，目的是为了提取更多的特征点能够更好的初始化地图
  - 深度和基线的参数设置
- 设置LocalMapper、LoopClosing、Viewer为了资源共享

## ==ORBextractor构造函数：==

- 确定金字塔每一层缩放的系数和$\sigma^2$
- 分配金字塔每一层特征点个数
- 关于描述子、旋转计算有关参数设置

## ==Frame构造函数：==

- 为双目相机准备的构造函数
- 为RGBD相机准备的构造函数
- 为单目相机准备的构造函数
  - 获取ORBextractor实例化的对象中设置的相关参数
  - 提取特征点、关键点、描述子，矫正去畸变
  - 分配点到网格图像中



## ==Tracking::Track()：==

利用状态机实现状态跳转

- ### 初始化

  - 对前两帧的图像初始化和特征点匹配（要求匹配的特征点数>100）
  - 完成初始化后再有图像传入就是做位姿估计
- ### 跟踪局部地图
- 记录位姿，用于轨迹复现





## LoopClosing线程工作内容

- 闭环检测，获得闭环候选帧集合；
- 几何验证，确定闭环帧，计算sim3,根据sim3的计算值更新地图点的位姿；
- 进行地图点融合和位姿优化；

以millisecond(5)的时间间隔执行主线程，其代码非常简洁：

```c++
if CheckNewKeyFrmes()
	if DetectLoop()
		if ComputeSim3()
			CorrectLoop()
```

```c++
bool LoopClosing::DetectLoop()
{
    /*
    @brief
    
    Step1:
	从队列取出最早进来的关键帧，并且回环间隔大于10帧
	
	Step2:
	取出所有当前帧的共视关键帧（>15个共视地图点）,计算关键帧的最小得分minScore
	
	Step3:
	KeyFrameDatabase::DetectLoopCandidates(pKF,minScore)筛选候选集
	
	Step4:
	连续性检测进一步筛选
		- 取候选帧构建候选帧组
			-遍历之前的候选帧组
				- 如果当前候选帧组中的帧存在于之前的候选帧组，我们成为连续，记录连续数量
				- 如连续数量超过阈值3，我们将这个关键帧放入候选帧集合
	
	*/
}
```

```c++
vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(pKF,minScore)
{
    /*
    
    @brief
    - 在关键帧数据库中根据公共的词汇和相似度阈值minScore筛选候选帧集合
    - 分组计算阈值minScoreToRetain，属于一个组的关键帧就只剩一个，进一步缩小候选集
    - 0.75*minScoreToRetain再筛选
    
    Step1:
    计算关键帧数据库中的关键帧的与当前帧的最大共视词汇数量，选取候选帧的第一步
        - 取出当前帧的局部相连的关键帧剔除，不参与回环
        - 遍历当前帧的词汇
        	- 遍历包含该词汇的关键帧
        		- 记录最大word个数，把有公共单词的关键帧push到SharingWords（候选回环帧）
	
	Step2:
	确定最小公共词汇数为最大公共词汇数的0.8倍
	
	Step3:
	遍历上述闭环候选帧SharingWords
		- 用mBowVec计算相似度得分
		- 挑出 词汇数量>最小词汇数 && 得分>minScore 的关键帧
		- 组成lScoreAndMatch候选帧集合
	
	@brief
	单独计算当前帧和某一个关键帧的相似性是不够的，这里将与关键帧相连（权值最高、共视程度最高）的前十个关键帧归为一组
	Step4:
	- 把候选帧的前十个共视帧组成一组
	- 计算每一组的总得分以及得分最高的帧和分
	- 计算这几组的最高分bestAccScore
	- 计算阈值minScoreToRetain = 0.75*bestAccScore
	
	Step5:
	- 取组中得分大于minScoreToRetain的候选关键帧集合，返回
	
	
	*/
}
```

```C
bool LoopClosing::ComputeSim3()
{
    /*
    @brief
    - 根据特征点数筛选候选帧集合，添加Sim3Solver求解器
    - Sim3求解，根据 内点数量&&是否收敛 进一步筛选候选帧集合，找出闭环帧
    - 检查闭环帧的共视关键帧和他们的地图点，根据Sim3变换，计算总的特征点的数量，根据阈值判断是不是闭环成功
    
    Step1:
    遍历候选帧集，通过词袋加速算法选出匹配的特征点数大于阈值20的候选帧集合，并为每一个候选帧构造一个Sim3Solver
    
    Step2：
    - 使用RANSAC法，对每一个候选帧求解候选帧到当前帧的sim3的变换
    - 如果没有收敛则剔除
    - 如果算出Sim3变换，求解更多的特征点匹配，因为SearchByBow匹配可能会有遗漏
    	-查找更多的地图点匹配，双向优化的方法
    	-优化Sim3，然后确定内点数量
    - 内点数大于20，找到闭环帧，停止迭代，算出世界坐标系w到候选帧m的sim3变换，w->c的sim3变换 
    
    Step3:
    取出闭环帧mpMatchedKF的共视关键帧vpLoopConnectedKFs，形成闭环帧小组，把这些共视关键帧的的地图点，放入mvpLoopMapPoints
    
    Step4:
    将闭环关键帧及其连接关键帧的所有地图点投影到当前关键帧进行投影匹配，根据sim变换，查找更多的匹配，成功的闭环匹配需要满足足够多的匹配特征点数
    
    Step5:
    统计当前帧和闭环帧（包括共视）匹配地图点数目，如果超过阈值40说明成功闭环，否则失败
    
    */
}
```

# sim3求解


单目SLAM在运行过程中，不但会积累旋转和平移上的误差，还会产生尺度上的漂移。ORB-SLAM2中，对于单目的尺度漂移，回环优化的时候会做特殊的处理，即sim3的优化。

基本问题为一些已知的点是在两个不同的坐标下的测量，目的是找到这两个坐标系的变换，和ICP不同的是，还需要多求一个尺度量。

<img src="C:\Users\djr\AppData\Roaming\Typora\typora-user-images\image-20211211114035238.png" alt="image-20211211114035238" style="zoom:50%;" />



## Definition:

两个坐标系的坐标分别为$r_{l,1}, r_{l,2}, r_{l,3}$和$r_{r,1}, r_{r,2}, r_{r,3}$，现在只讨论左坐标系，

- X轴：$x^e=x_l/||x_l||, x_{l}=r_{l,2}-r_{l,1}$
	
- Y轴：$y^e=y_l/||y_l||, y_l=(r_{l,3}-r_{l,1})-[(r_{l,3}-r_{l,1})*x_{l}^e]x_l^e$
	
- Z轴：$z^e=x^e×y^e$

得到左右坐标系的基底：

- $M_l=[x^e_l,y^e_l,z^e_l],M_r=[x^e_r,y^e_r,z^e_r]$

假设左边坐标系有一向量$r_l$，那么变换到右坐标系为:

- $r_r=M_rM_l^Tr_l$
- $R=M_rM_l^T$

假设有n个点，我们可以找到：

- $r_r=sR(r_l)+r_0,||R(r_l)||^2=||r_l||^2,r_0是平移量$，

构造误差模型：

- $e_i=r_{r,i}-sR(r_{l,i})-r_0$

最小误差平方和模型：


- $\Sigma_{i=1}^n||e_i||^2$

  

## 计算平移量

计算的左右坐标系中所有点的质心：

- $\hat{r_l}=\frac{1}{n}\sum\limits_{i=1}^nr_{l,i}$
- $\hat{r_r}=\frac{1}{n}\sum\limits_{i=1}^nr_{r,i}$

则每一个点距离质心的距离为：

- $r_{l,i}^{\prime}=r_{l,i}-\hat{r_l}$
- $r_{r,i}^{\prime}=r_{r,i}-\hat{r_r}$
- $\sum\limits_{i=1}^{n}r^{\prime}_{l,i}=0$
- $\sum\limits_{i=1}^{n}r^{\prime}_{r,i}=0$

变换公式：

- $r^{\prime}_{r,i}=sR(r^{\prime}_{l,i})+r^{\prime}_0$

$$
\begin{align*}
r^{\prime}_0
& = r^{\prime}_{r,i}-sR(r^{\prime}_{l,i}) \\
& = r_{r,i}-\hat{r_r} - sR(r_{l,i}-\hat{r_l}) \\
& = r_{r,i}-\hat{r_r} - sR(r_{l,i})+sR(\hat{r_l}) \\
& = r_0-\hat{r_r}+sR(\hat{r_l})
\end{align*}
$$

优化函数：

- $\sum\limits_{i=1}^n||e_i||^2=\sum\limits_{i=1}^n||r_{r,i}-sR(r_{l,i})-r_0||^2$
- 算是两种优化函数，注意不是等价过来的
- $\sum\limits_{i=1}^n||e_i||^2=\sum\limits_{i=1}^n||r_{r,i}^{\prime}-[sR(r_{l,i}^{\prime})+r_0^{\prime}]||^2$
- $\sum\limits_{i=1}^n||e_i||^2=\sum\limits_{i=1}^n||r_{r,i}^{\prime}-sR(r_{l,i}^{\prime})||^2+n||r_0^{\prime}||^2,2r_0^{\prime}\sum\limits_{i=1}^n[r_{r,i}^{\prime}-sR(r_{l,i}^{\prime})]=0$

注意到我们的优化函数为非负，且只有$r_0^{\prime}=0$的情况下取得最小值，带入下面的式子：

$$
\begin{align*}
r^{\prime}_0
& = r^{\prime}_{r,i}-sR(r^{\prime}_{l,i}) \\
& = r_{r,i}-\hat{r_r} - sR(r_{l,i}-\hat{r_l}) \\
& = r_{r,i}-\hat{r_r} - sR(r_{l,i})+sR(\hat{r_l}) \\
& = r_0-\hat{r_r}+sR(\hat{r_l})
\end{align*}
$$
有平移量$t=r_0=\hat{r_r}-sR(\hat{r_l})$

## 尺度计算，两种情况的表达形式

由于$r_0^{\prime}=0$,所以可得误差函数：

$$
\begin{align*}
\sum\limits_{i=1}^n||e_i||^2
& =\sum\limits_{i=1}^n||r_{r,i}^{\prime}-[sR(r_{l,i}^{\prime}+r_0^{\prime})]||^2 \\
& =\sum\limits_{i=1}^n||r_{r,i}^{\prime}-sR(r_{l,i}^{\prime})||^2 \\
& = \sum\limits_{i=1}^n||r_{r,i}^{\prime}||^2-2s\sum\limits_{i=1}^nr_{r,i}^{\prime}R(r_{l,i}^{\prime})+s^2\sum\limits_{i=1}^n||R(r_{l,i}^{\prime})||^2 \\
记& = S_r-2sD+s^2S_l \\
& =(s\sqrt{S_l}-D/\sqrt{S_l})^2+(S_rS_l-D^2)/S_l
\end{align*}
$$
要是误差函数最小值，第一项必须为0，此时：

- $s=D/S_l=[\sum\limits_{i=1}^nr_{r,i}^{\prime}R(r_{l,i}^{\prime})]/\sum\limits_{i=1}^n||R(r_{l,i}^{\prime})||^2$

如果两组测量中的误差相似，则对误差项使用对称表达式更为合理：

- 原式$e_i=r_{r,i}^{\prime}-sR(r_{l,i}^{\prime})-r_0^{\prime}=r_{r,i}^{\prime}-sR(r_{l,i}^{\prime})$
- 做根号s的变换为$e_i=\frac{1}{\sqrt{S}}r_{r,i}^{\prime}-\sqrt{s}R(r_{l,i}^{\prime})$
- $\sum\limits_{i=1}^n||e_i||^2=(\sqrt{s}S_l-\frac{1}{\sqrt{s}}S_r)^2+2(S_lS_r-D)$

求改式最小值，得：

- $s=(\frac{\sum\limits_{i=1}^n||r_{r,i}^{\prime}||^2}{\sum\limits_{i=1}^n||r_{l,i}^{\prime}||^2})^\frac{1}{2}$

从这个表达式我们可以看出，他的结果不依赖于旋转，也就是说允许人们在不知道旋转的情况下确定尺度。



## 计算R

- 为了求解最小误差项，我们应该去求最大的$D=\sum\limits_{i=1}^nr_{r,i}^{\prime}R(r_{l,i}^{\prime})$
- 引入四元数来表达

$$
\begin{align*}
D&=\sum\limits_{i=1}^nr_{r,i}^{\prime} \cdot R(r_{l,i}^{\prime}),我们要清楚这里是内积，满足交换律\\
&=\sum\limits_{i=1}^n R(r_{l,i}^{\prime}) \cdot r_{r,i}^{\prime}\\
&=\sum\limits_{i=1}^n q\mathring{r_{l,i}^{\prime}}q^* \cdot \mathring{r_{r,i}^{\prime}}\\
&=\sum\limits_{i=1}^n q\mathring{r_{l,i}^{\prime}} \cdot \mathring{r_{r,i}^{\prime}}q \\
&=\sum\limits_{i=1}^n \overline\R_{l,i} q \cdot \R_{r,i}q\\
没懂&=\sum\limits_{i=1}^n q^T\overline\R_{l,i}^T  \cdot \R_{r,i}q\\
&=q^TNq\\
\end{align*}
$$

N是对称矩阵，其中的元素用坐标表示，这里省略表示形式，具体看论文

对N矩阵进行特征值分解求解最大特征值对应的特征向量，就是q，==不是很理解==

- q已知，可求欧拉角、R，然后再求平移量t和尺度s



## 闭环

将当前相机的位姿和场景与回环帧处的位姿和场景进行融合，从而消除累计误差，矫正运动轨迹。



<img src="C:\Users\djr\AppData\Roaming\Typora\typora-user-images\image-20211213140312972.png" alt="image-20211213140312972" style="zoom:67%;" />


```C++
void LoopClosing::CorrectLoop()
{
    /*
    @brief
    
	Step0:
	等局部地图结束，结束BA线程，为闭环做准备
	
	Step1:
	更新当前关键帧与其它关键帧之间的连接关系
	
	Step2:
		- mg2oScw=gScm*gSmw 闭环帧到当前帧的转换，世界坐标系，属于优化的，gScm为sim3求解优化的
		- 固定当前帧，求解闭环帧组(共视)的优化位姿和无优化位姿，指的是w->m的坐标变换
		- 遍历共视帧的地图点
			- 将该未校正的eigP3Dw先从世界坐标映射到未校正的pKFi相机坐标系，然后再反射到校正的世界坐标系下
			- 达到更新pMPi世界坐标的目的
		- 更新闭环共视帧组的信息，位置、权值等
	
	Step3:
		- 检查当前帧的地图点有无与投影匹配上的点，对匹配的的点进行替换(优化的地图点更精确)，或者填补，因为现有的地图点很可能存在累计误差
		- 疑问：闭环投影的点有没有可能在当前帧不存在
		
	Step4:
		- 将这些已纠正位姿的MapPoints与闭环MapPoints融合
		
	Step5:
		- 更新关键帧的连接关系
	
	Step6:
		- 本质图优化,Optimizer.cc文件中的优化器都是静态函数
		- OptimizeEssentialGraph()
		
	Step7:
		- 互相添加回环边
		
	Step8:
		- BA
	
	*/
}
```



## Optimizer



## 计算相机运动——前端，个人理解根据帧与帧之间的位姿估计，优化单个的位姿

- ### 单目，根据两组2D点估计运动，对极几何解决

- ### RGBD、双目，根据两组3D点估计运动，通常用ICP解决

- ### 如果一组为3D，一组为2D，该问题通过PnP（DLT、P3P、EPnP、UPnP）求解，RGBD、双目



## 对几极何

- $s_1p_1=KP,s_2p_2=K(RP+t)$
- 尺度意义下的相等,$sp\simeq p$
- $p_1\simeq KP,p_2\simeq K(RP+t)$
- $x_1=K^{-1}p_1,x_2=K^{-1}p_2$
- $x_2\simeq K^{-1}p_2 \simeq RP+t \simeq Rx_1+t$
- 左右做 $t\times$ 处理，$t\times x_2 \simeq t\times Rx_1 + 0$
-  左右乘$x^T_2$，$x^T_2t\times x_2 \simeq x^T_2t\times Rx_1$
- $x^T_2t\times Rx_1=0$
- $p^T_2x^T_2t\times Rx_1p_1=0$，记为对极约束
- 进一步简化对极约束，本质矩阵 $E=t\times R$ ，基础矩阵$F=K^{-T}EK^{-1}$，$x_2^TEx_1=p_2^TFp_1=0$
- 根据配对的像素点求$F\to E\to R,t$

### 本质矩阵E的求解,$F$也是这么求解的

- 八点法才能求解$E$
- 由奇异值的分解$SVD$恢复相机运动的$R,t$
  - $E=U\sum V^T$



## PnP(Perspective-n-Point)，描述了当知道n个3D空间点及其投影位置时，如何估计相机的位姿。

### 直接线性变换DLT，最少匹配6对点实现矩阵的线性求解

### P3P

### 最小化重投影误差求解PnP（BA）





## 后端——以全局的重投影误差来优化每一帧的位姿，五种优化方式

- ### 卡尔曼滤波器
- ### 扩展卡尔曼滤波器
- ### BA：把一路上所有的坐标点与位姿整体放在一起作为自变量进行非线性优化
- ### Pose Graph：在优化几次以后把特征点固定住不再优化，只当做位姿估计的约束，之后主要优化位姿。
- ### 因子图，因子图是增量的处理后端优化。机器人是运动的，不同的边和节点会不断加入图中，每加入一个点，普通图优化是对整个图进行优化，比较麻烦和耗时，因子图相当于保留中间结果，每加入一个点，只计算需要计算的点即可，加速计算，避免冗余计算。
- ### 在ORBSLAM2中，本质图优化，局部BA，关键帧只和回环相关，构成本质图
>>>>>>> 5bf7e32f7c1888a8d0b3b9be4f97775fb5fa288c
