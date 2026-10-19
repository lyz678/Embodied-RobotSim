# ACT 厨房抓取结果

Gazebo 厨房可乐任务，新增60条成功示范，与原三类数据混合训练。新增数据按42/9/9划分；ResNet18＋类别one-hot，seed42、fp32。成功要求导航通过、指定可乐实际抬起6cm、持物回Home连续保持2模拟秒。以下结果在局部／全局OctoMap拆分之前获得。

[rgb train／val loss 曲线](/home/lyz/ACT/checkpoints/gazebo_kitchen_rgb/loss_curves.png)
[rgbd train／val loss 曲线](/home/lyz/ACT/checkpoints/gazebo_kitchen_rgbd/loss_curves.png)
[rgbxyz train／val loss 曲线](/home/lyz/ACT/checkpoints/gazebo_kitchen_rgbxyz/loss_curves.png)

## 训练与闭环评估汇总

60条厨房可乐成功示范采集完毕；三组各训练20000步，batch16。冻结权重在10个独立停车种子上分别测试，未将验收数据用于训练。

| 模型 | 导航通过 | 实际抬起 | 持物回Home | 完整任务成功率 |
|---|---|---|---|---|
| rgb | 9/10 | 3/10 | 2/10 | 2/10（20%） |
| rgbd | 10/10 | 9/10 | 4/10 | 4/10（40%） |
| rgbxyz | 10/10 | 6/10 | 2/10 | 2/10（20%） |

RGB＋深度目前最好，抬起9/10，但只有4/10满足持物回Home连续保持的物理标准。仅在线闭爪/Home判定不计作完整成功；每组样本仅10次，不宜把差异解释为普遍结论。尚未达到8/10标准。

完成产物共379项文件哈希核验通过。

| 回合 | RGB视频 | RGB＋深度视频 | RGB＋XYZ视频 |
|---|---|---|---|
| 0 | [失败](../../../log/embodied_act/kitchen_training/test_00_rgb/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_00_rgb/score.json) | [成功](../../../log/embodied_act/kitchen_training/test_00_rgbd/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_00_rgbd/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_00_rgbxyz/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_00_rgbxyz/score.json) |
| 1 | [失败](../../../log/embodied_act/kitchen_training/test_01_rgb/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_01_rgb/score.json) | [成功](../../../log/embodied_act/kitchen_training/test_01_rgbd/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_01_rgbd/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_01_rgbxyz/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_01_rgbxyz/score.json) |
| 2 | [成功](../../../log/embodied_act/kitchen_training/test_02_rgb/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_02_rgb/score.json) | [成功](../../../log/embodied_act/kitchen_training/test_02_rgbd/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_02_rgbd/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_02_rgbxyz/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_02_rgbxyz/score.json) |
| 3 | [失败](../../../log/embodied_act/kitchen_training/test_03_rgb/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_03_rgb/score.json) | [成功](../../../log/embodied_act/kitchen_training/test_03_rgbd/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_03_rgbd/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_03_rgbxyz/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_03_rgbxyz/score.json) |
| 4 | [失败](../../../log/embodied_act/kitchen_training/test_04_rgb/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_04_rgb/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_04_rgbd/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_04_rgbd/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_04_rgbxyz/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_04_rgbxyz/score.json) |
| 5 | [成功](../../../log/embodied_act/kitchen_training/test_05_rgb/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_05_rgb/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_05_rgbd/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_05_rgbd/score.json) | [成功](../../../log/embodied_act/kitchen_training/test_05_rgbxyz/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_05_rgbxyz/score.json) |
| 6 | [失败](../../../log/embodied_act/kitchen_training/test_06_rgb/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_06_rgb/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_06_rgbd/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_06_rgbd/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_06_rgbxyz/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_06_rgbxyz/score.json) |
| 7 | [失败](../../../log/embodied_act/kitchen_training/test_07_rgb/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_07_rgb/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_07_rgbd/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_07_rgbd/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_07_rgbxyz/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_07_rgbxyz/score.json) |
| 8 | [失败](../../../log/embodied_act/kitchen_training/test_08_rgb/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_08_rgb/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_08_rgbd/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_08_rgbd/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_08_rgbxyz/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_08_rgbxyz/score.json) |
| 9 | [失败](../../../log/embodied_act/kitchen_training/test_09_rgb/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_09_rgb/score.json) | [失败](../../../log/embodied_act/kitchen_training/test_09_rgbd/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_09_rgbd/score.json) | [成功](../../../log/embodied_act/kitchen_training/test_09_rgbxyz/task_rgb.mp4) · [评分](../../../log/embodied_act/kitchen_training/test_09_rgbxyz/score.json) |

## 与 YOLOE＋GraspNet＋MoveIt 的已有记录对比

统计全部采集尝试，包含失败；不能用保存的60条成功示范作为分母。完整成功要求导航通过、指定可乐实际抬起、持物回Home连续保持2模拟秒。

| 链路 | 导航通过 | 完整任务成功 | 到达厨房后的成功率 |
|---|---|---|---|
| YOLOE＋GraspNet＋MoveIt（全部采集） | 94/110 | 60/110（54.5%） | 60/94（63.8%） |
| ACT RGB | 9/10 | 2/10（20%） | 2/9（22.2%） |
| ACT RGB＋深度 | 10/10 | 4/10（40%） | 4/10（40%） |
| ACT RGB＋XYZ | 10/10 | 2/10（20%） | 2/10（20%） |

传统链路恢复前34/75（45.3%），延长位姿采样窗口并增加启动等待后26/35（74.3%）。这两个阶段的场景种子不同，不能将差异全部归因于修复。全部110次中16次未通过导航，另34次到达后未完成抓取任务。

这只是已有记录对照：传统链路用61000起的采集种子且累计60条成功后停止，ACT用71000起的10个独立验收种子；传统链路还经历过配置调整。任务物理门槛一致，但传统采集记录器与ACT离线评分的采样实现不同。因此不能视为严格配对消融或从这些数值直接推断显著优势。严格比较应将冻结传统链路在71000–71009上重新测试，并使用同一离线评分器。

[完整统计与110次任务明细](../../../log/embodied_act/kitchen_training/traditional_vs_act.json)

## 历史结果

厨房数据重训前，Gazebo迁移测试RGB 0/7、RGB＋深度0/6、RGB＋XYZ 0/6；下一回合因证据缺失停止。旧Isaac厨房实验仅完整完成1回合，失败后中止，不与Gazebo合并统计。

[旧迁移实验](../../../log/embodied_act/gazebo_evaluation/summary.json) · [延迟诊断](../../../log/embodied_act/gazebo_evaluation/latency_diagnosis.json)
