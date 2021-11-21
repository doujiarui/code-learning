# ORB-SLAM 3

## Abstract

第一个提出视觉、视觉惯导、多地图单目、双目、深度相机的SLAM框架。

创新点：
-  基于特征提取的紧耦合VI SLAM系统，具有实时鲁棒性，比之前的方法精确2-10倍
- 高调用率的多地图位置识别系统（一旦丢失会建立一张新的地图，当重识别时会与之前的地图融合）

## INTRODUCTION

介绍SLAM目前流行的方案

MAP：视觉传感器对应的是BA（geometric BA minimzies feature reprojection error \ photometric BA minizes the photometric error of a set of seleced pixels）

three tpes of data association ==[1]==:
- short-term：短时间内地图匹配，容易丢失或者漂移当移动到相似的地方 
- mid-term：回环+BA优化，精度更好，reach zero drift
- long-term：用位置识别去匹配先前的观测，==跟回环的区别？==
   （==Visual Place Recognition、https://blog.csdn.net/Kevin_cc98/article/details/81462490==）

该工作基于ORB-SLAM【2】【3】，==Visual-Inerial【4】==

而且进一步提出==multi-map data association==

该系统的新颖之处在于:
- 基于VI,在IMU初始化阶段就用MAP,初始化的方法由 ==【6】==
- Improved-recall place recognition.最近的slam系统【2、7、8】解决位置识别都用DBoW2词袋库 ==【9】==，DBoW2需要时间一致性，在检查几何一致性之前，将三个连续的关键帧匹配到同一区域，以牺牲召回率为代价来提高精度。因此，系统在关闭循环和重用以前映射的区域时速度太慢。他们首先检查候选关键帧的几何一致性，然后检查与三个共可见关键帧的局部一致性，在大多数情况下已经在地图中。这种策略增加了召回率，并增加了数据关联，提高了地图的准确性，但代价是略高的计算成本。
- Atlas：a set of disconnected maps. 地图集的初步版本： ==【10】==
- An abstract camera representation。一种抽象的相机表示，使SLAM代码与所使用的相机模型无关，并允许通过提供投影、非投影和雅可比矩阵函数来添加新的模型。我们提供了针孔[11]和鱼眼[12]模型的实现。

## RELATED WORK

