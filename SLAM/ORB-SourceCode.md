# ORBSLAM2_SourceCode

<img src="C:\Users\djr\AppData\Roaming\Typora\typora-user-images\image-20211206104441902.png" alt="image-20211206104441902" style="zoom:80%;" />

## 不管以哪种方式运行SLAM都会实例化==System.cc：==

1.  确定传感器类型并读取ORB字典，为后期的回环检测做准备；
2.  创建关键帧数据库KeyFrameDatabase，用于存放关键帧相关数据；
3.  初始化Tracking线程。其实Tracking线程是在main线程中运行的，可以简单的认为main线程就是Tracking线程；
4.  初始化并启动LocalMapping线程；
5.  初始化并启动LoopClosing线程；
6.  初始化并启动窗口显示线程mptViewer；
7.  在各个线程之间分配资源，方便线程彼此之间的数据交互。
8.  实例化地图

根据传感器类型传入带有不同参数的图片（左右图、深度等），调用Tracking线程名为== GrabImag+相机类型==的函数， 然后返回相机的位姿（世界坐标系到该帧相机坐标系的变换矩阵）

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

- 闭环检测，获得闭环候选帧；
- 计算sim3,根据sim3的计算值更新地图点的位姿；
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
	1、从队列取出最早进来的关键帧，并且回环间隔大于10帧
	2、取出所有当前帧的共视关键帧（>15个共视地图点）
	3、计算关键帧的最小得分minScore
	4、从关键帧数据库中找不低于minScore的候选帧
	5、KeyFrameDatabase::DetectLoopCandidates(pKF,minScore)筛选候选集
	6、连续性检测
}
```

1. 词汇
2. 得分和词汇*0.8
3. 分组计算阈值minScoreToRetain，属于一个组的关键帧就只剩一个，进一步缩小候选集
4. 0.75*minScoreToRetain再筛选

```c++
KeyFrameDatabase::DetectLoopCandidates()
{
    1、计算关键帧数据库中的关键帧的与当前帧的共视词汇数量，选取候选帧的第一步
        - 取出当前帧的局部相连的关键帧
        - 遍历当前帧的词汇
        - 遍历包含该词汇的关键帧
        - 记录word个数，push到SharingWords（候选回环帧）
	2、确定最小公共词汇数为最大公共词汇数的0.8倍
	3、遍历上述闭环候选帧，挑出 词汇数量>最小词汇数 && 得分>minScore 的关键帧
	4、单独计算当前帧和某一个关键帧的相似性是不够的，这里将与关键帧相连（权值最高、共视程度最高）的前十个关键         帧归为一组，记录每一组的最高分，得最高分的关键帧，确定阈值minScoreToRetain = 0.75*bestAccScore
	5、取组中得分大于minScoreToRetain的关键帧,并返回
}
```