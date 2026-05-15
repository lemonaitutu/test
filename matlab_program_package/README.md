# MATLAB 程序包说明

本程序包根据 `程序.pdf` 中的 MATLAB 程序部分整理而成，并在原捷联惯导程序基础上增加了里程计辅助导航和平台式惯导/里程计辅助导航示例。

## 文件结构

- `run_all.m`：依次运行全部示例。
- `test_8_2_2.m`：捷联惯导纯惯性仿真主程序。
- `test_8_3_2.m`：捷联惯导/里程计辅助导航仿真主程序，采用里程计前向速度和非完整约束，不使用 GPS。
- `test_platform_odo.m`：平台式惯导/里程计辅助导航仿真主程序。
- `run_platform_odo_1day.m`：运行新增的一天仿真版本。
- `test_platform_odo_1day_static.m`：平台式惯导/里程计辅助的静基座一天仿真。
- `test_platform_odo_1day_east20.m`：平台式惯导/里程计辅助的东向 20 m/s 一天仿真。
- `platform_odo_1day_core.m`：一天仿真的公共核心函数，负责误差统计、输出和绘图。
- `insupdate.m`：捷联惯导机械编排更新。
- `pnsupdate.m`：平台式惯导机械编排更新，平台系 `p` 近似跟踪导航系 `n`。
- `gvar.m`：常用单位与地球参数全局变量。
- `earth.m`、`imuadderr.m`：地球参数计算与 IMU 误差注入。
- `kfinit.m`、`kfupdate.m`、`kff15.m`：Kalman 滤波初始化、更新与 15 维误差模型。
- `a2*.m`、`m2*.m`、`q*.m`、`rv2*.m`：姿态矩阵、四元数、旋转矢量等转换工具。
- `msplot.m`、`deltapos.m`：绘图与位置误差显示辅助函数。

## 使用方法

在 MATLAB 中进入本文件夹后运行：

```matlab
run_all
```

也可以分别运行：

```matlab
test_8_2_2
test_8_3_2
test_platform_odo
run_platform_odo_1day
```

## 平台式惯导/里程计说明

`test_platform_odo.m` 中，平台式惯导使用 `qnp` 表示平台坐标系 `p` 到导航坐标系 `n` 的姿态误差。加速度计输出被视为平台系测量值，经 `qnp` 投影到导航系后完成速度和位置更新。里程计量测仍采用车体系速度约束：

```matlab
vb = Cbn0*vn;
odo = Cbn0*vn0 + rk.*randn(3,1);
kf.Hk = [zeros(3), Cbn0, zeros(3,9)];
```

因此该示例不使用 GPS 速度或位置，而是通过里程计前向速度和横向/垂向非完整约束修正惯导速度误差，并由系统误差模型间接抑制姿态、位置和器件误差漂移。

## 一天仿真输出

`run_platform_odo_1day.m` 会依次运行两个新增场景：

- 静基座，仿真时间 24 h。
- 东向 20 m/s 匀速运动，仿真时间 24 h。

每个场景会在命令行输出以下误差均方值：

- 姿态误差均方值，单位 `(arcmin)^2`。
- 速度误差均方值，单位 `(m/s)^2`。
- 位置误差均方值，单位 `m^2`。
- 陀螺零偏估计误差均方值，单位 `(deg/h)^2`。
- 加速度计零偏估计误差均方值，单位 `ug^2`。

同时会保存结果文件：

- `platform_odo_1day_static_results.mat`
- `platform_odo_1day_east20_results.mat`
