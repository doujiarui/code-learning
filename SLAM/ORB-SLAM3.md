# ORB-SLAM 3

## Abstract

第一个提出视觉、视觉惯导、多地图单目、双目、深度相机和鱼眼相机的SLAM系统。

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

首先对比了各种指标，如图表1所示。

### A.Visual SLAM 【13、14、52】 EKF+ShiTomasi Points跟踪

    BA优化。最具代表性的系统是PTAM[16]，该系统将摄像机跟踪和映射分为两个平行线程。在计算成本相同的情况下，基于关键帧的技术比过滤更精确[55]，成为视觉SLAM和VO的黄金标准。在[56]中使用滑动窗口BA实现了大规模单目SLAM，在[57]中使用双窗口优化和共视图实现了大规模单目SLAM。
    基于这些思想，ORB-SLAM[2]，[3]使用ORB特性，其描述符提供短期和中期数据关联，构建共视图以限制跟踪和映射的复杂性，并使用词包库DBoW2[9]执行循环关闭和重新定位，实现长期数据关联。到目前为止，它是唯一一个集成了三种数据关联的视觉SLAM系统，我们相信这是其卓越准确性的关键。在这项工作中，我们使用新的Atlas系统提高了其在纯视觉SLAM中的鲁棒性，该系统在跟踪丢失时启动新的地图，并使用具有改进召回率的新位置识别方法提高了其在循环场景中的准确性。
    直接方法不提取特征，而是直接使用图像中的像素强度，并通过最小化光度误差来估计运动和结构。LSD-SLAM[20]能够使用高梯度像素构建大比例尺半密集地图。然而，map估计被简化为posegraph（PG）优化，其精度低于PTAM和ORB-SLAM[2]。混合系统SVO[23]，[24]提取快速特征，使用直接方法跟踪特征和帧与帧之间具有非零强度梯度的任何像素，并使用重投影误差优化相机轨迹和3D结构。SVO非常有效，但作为纯VO方法，它只执行短期数据关联，这限制了其准确性。直接稀疏里程计DSO[27]能够在点探测器性能不佳的情况下计算精确的相机姿态，增强低纹理区域或模糊图像的鲁棒性。它引入了局部光度BA，可同时优化包含七个最近关键帧的窗口和点的反向深度。这项工作的扩展包括立体声[29]，使用功能和DBoW2[58][59]闭合环路，以及视觉惯性里程计[46]。直接稀疏映射DSM[31]引入了直接方法中映射重用的思想，显示了中期数据关联的重要性。在所有情况下，缺乏短期、中期和长期数据关联的整合导致精度低于我们的提案（见第七节）。

## Visual-Inertial SLAM

    视觉传感器和惯性传感器的结合提供了对不良纹理、运动模糊和遮挡的鲁棒性，并且在单目系统的情况下，使尺度可观察。紧耦合方法的研究可以追溯到MSCKF[33]，其中通过特征边缘化避免了特征数量的EKF二次成本。最初的系统在[34]中得到完善，并在[35]、[36]中扩展到立体声。第一个基于关键帧和束调整的紧密耦合视觉里程计系统是OKVIS[38]，[39]，它也能够使用单目和立体视觉。虽然这些系统依赖于特征，但ROVIO[41]，[42]使用直接数据关联向EFK提供光度误差。
    ORB-SLAM-VI[4]首次提出了一种视觉-惯性SLAM系统，该系统能够重用具有短期、中期和长期数据关联的地图，并在基于IMU预积分的精确局部视觉惯性BA中使用它们[60]，[61]。然而，它的IMU初始化技术太慢，需要15秒，这损害了鲁棒性和准确性.[62]，[63]中提出了更快的初始化技术，该技术基于一个封闭形式的解决方案，用于联合检索比例、重力、加速计偏差和初始速度\视觉特征深度.
    关键的是，它们忽略了IMU噪声特性，并最小化了空间点的3D误差，而不是它们的重投影误差，这是基于特征的计算机视觉的黄金标准。我们之前的工作[64]表明，这会导致大量不可预测的错误。
    VINS Mono[7]是一个非常精确和稳健的单目视觉里程计系统，具有使用DBoW2和4自由度姿势图优化的闭环，以及地图合并。特征跟踪是使用Lucas Kanade tracker执行的，比描述符匹配稍微健壮一些。在VIN核聚变[44]中，它已扩展到立体和立体惯性。
    VI-DSO[46]将DSO扩展到视觉惯性里程计，提出了一种捆绑调整，将惯性观测与选定高梯度像素的光度误差相结合，从而获得非常好的精度。成功地利用了高梯度像素的信息，增强了纹理较差场景区域的鲁棒性。他们的初始化方法依赖于视觉惯性BA，需要2030秒才能在1%的标度误差范围内收敛。
    最近的玄武岩[47]是一种立体惯性里程计系统，它从视觉惯性里程计中提取非线性因素，用于BA，并闭合匹配ORB特征的环路，实现非常好到极好的精度。
     Kimera [8]是一种新型的优秀的度量语义映射系统，但其度量部分包括立体惯性里程计加上DBoW2闭环和姿势图优化，实现了与VINS Fusion相似的精度
     在这项工作中，我们建立在ORB-SLAM-VI上，并将其扩展到立体惯性SLAM。我们提出了一种新的基于最大后验概率（MAP）估计的快速初始化方法，该方法适当地考虑了视觉和惯性传感器的不确定性，并在2秒内以5%的误差估计真实尺度，在15秒内收敛到1%的尺度误差。上面讨论的所有其他系统都是视觉惯性里程计方法，其中一些系统通过环路闭合进行扩展，并且缺乏使用中期数据关联的能力。我们相信，这一点，加上我们快速而精确的初始化，是我们的系统获得更高精度的关键，即使在没有循环的序列中也是如此。

## Multi-Map SLAM

    在[65]中，在滤波方法中首次提出了通过地图创建和融合来增加勘探过程中跟踪损失的鲁棒性的想法。第一个基于关键帧的多贴图系统是[66]，但贴图初始化是手动的，系统无法合并或关联不同的子贴图。多地图功能已作为协作地图系统的一个组成部分进行了研究，包括多个地图代理和一个仅接收信息的中央服务器[67]，或具有双向信息流，如C2TAM[68]。MOARSLAM[69]为协作式多设备SLAM提出了一种健壮的无状态客户机-服务器体系结构，但主要关注的是软件体系结构，没有报告准确性结果
    最近，CCM-SLAM[70]，[71]提出了一种分布式多地图系统，用于具有双向信息流的多无人机，构建在ORB-SLAM的基础上。他们的重点是克服有限带宽和分布式处理的挑战，而我们的重点是准确性和鲁棒性，在EuRoC数据集上取得显著更好的结果。Slam[72]还提出了ORB-SLAM2的多地图扩展，但将子地图保留为独立实体，同时执行无缝地图合并，构建更精确的全局地图。
    VINS Mono[7]是一种视觉里程计系统，具有闭环和多地图功能，依赖于位置识别库DBoW2[9]。我们的实验表明ORB-SLAM3，由于能够使用中期数据关联，EuRoc数据集上的单目惯性单次会话操作的精度是VINS Mono的6倍。我们的Atlas系统也基于DBoW2，但提出了一种新的更高召回率的地点识别技术，并使用局部BA执行更详细和准确的地图合并，将优势增加到3。在EuRoC上的多会话操作中，精度是VINS Mono的2倍

## System Overview

ORBSLAM3 基于 ==Visual-inertial monocular SLAM with map reuse ORBSLAM 4【4】== 和 ORBSLAM2。

![image-20211122105442170](C:\Users\djr\AppData\Roaming\Typora\typora-user-images\image-20211122105442170.png)

和ORBSLAM2相似，但有一些新颖之处：
- Atlas，用到不同的DBoW2词袋库，用于重定位、回环和地图融合
- Tracking thread，处理传感器的信息并计算当前帧的位姿，最小化重投影误差去匹配地图的特征，而且还要判断是不是关键帧。当丢失时，会在altas重定位，将匹配到的地图成为active map，如果匹配不到alts，会将当前活跃的地图存储，在初始化一张新的地图。 
- Local mapping thread，adds keyframes and points to the active map, removes the redundant ones, and refines the map using visual or visual-inertial bundle adjustment, operating in a local window of keyframes close to the current frame. Additionally, in the inertial case, the IMU parameters are initialized and refined by the mapping thread using our novel MAP-estimation technique.
- loop and map merging thread detects common regions between the active map and the whole Atlas at keyframe rate. If the common area belongs to the active map, it performs loop correction; if it belongs to a different map, both maps are seamlessly merged into a single one, that becomes the active map. After a loop correction, a full BA is launched in an independent thread to further refine the map without affecting real-time performance.有时会在启动一个BA的线程去优化。

## CAMERA MODEL

ORB-SLAM在所有系统组件中均假设为针孔相机模型。我们的目标是通过将与相机模型相关的所有属性和函数（投影和非投影函数、雅可比矩阵等）提取到单独的模块中，从整个SLAM管道中提取相机模型。这允许我们的系统通过提供相应的摄像头模块来使用任何摄像头型号。在ORB-SLAM3库中，除了针孔模型（单目、双目、深度）外，我们还提供了Kannala Brandt[12]鱼眼模型。

### A.Relcalization

    一个健壮的SLAM系统需要在跟踪失败时重新定位摄像机的能力。ORB-SLAM通过设置基于ePnP算法[73]的透视n点解算器来解决重新定位问题，该算法假设在所有公式中都有一个校准的针孔相机。为了跟进我们的方法，我们需要一个PnP算法，该算法独立于使用的摄像机模型。因此，我们采用了最大似然透视n点算法（MLPnP）[74]，该算法与相机模型完全解耦，因为它使用投影光线作为输入。相机模型只需要提供从像素到投影光线的非投影功能，就可以使用重新定位。

### B.Non-rectified Stereo SLAM
    没看懂他们的解决方案

## VISUAL-INERTIAL SLAM

ORB-SLAM-VI[4]是第一个能够重用地图的真正视觉惯性SLAM系统。然而，它仅限于针孔单目相机，初始化速度太慢，在一些具有挑战性的场景中失败。在这项工作中，我们以ORB-SLAM-VI为基础，提供了一种快速、准确的IMU初始化技术，以及一个开源的SLAM库，能够使用针孔和鱼眼相机进行单目惯性和立体惯性SLAM。

### A.Fundamentals  

   【60、61】Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built Environments Without Initial Conditions 和 On-Manifold Preintegration for Real-Time Visual–Inertial Odometry

### C.Tracking and Mapping

   方案【4】Visual-Inertial Monocular SLAM with Map Reuse
   对于跟踪和映射，我们采用了[4]中提出的方案。跟踪解决了一个简化的视觉惯性优化问题，其中仅优化最后两帧的状态，而贴图点保持不变。对于映射，尝试从方程4中求解整个优化对于大型映射来说是困难的。我们使用关键帧及其点的滑动窗口作为可优化变量，还包括从可共视关键帧对这些点的观察，但保持其姿势固定。

### D.Robustness to tracking loss


## MAP MERGING AND LOOP CLOSING



在这项工作中，我们提出了一种新的地方识别算法，改进召回长期和多地图数据关联。每当贴图线程创建新的关键帧时，就会启动位置识别，尝试检测与Atlas中已存在的任何关键帧的匹配。如果找到的匹配关键帧属于活动贴图，则执行循环闭合。否则，它是一个多地图数据关联，然后将活动地图和匹配地图合并。作为我们方法中的第二个新颖之处，一旦估计了新关键帧和匹配贴图之间的相对姿势，我们定义了一个局部窗口，其中包含匹配关键帧及其在共视图中的邻居。在这个窗口中，我们集中搜索中期数据关联，提高循环结束和地图合并的准确性。这两个新特性解释了在EuRoC实验中，ORB-SLAM3比ORB-SLAM2获得更好的精度。下面将解释不同操作的细节。

