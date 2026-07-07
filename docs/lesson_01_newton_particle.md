# 第一讲：从质点 Newton 方程到数值求解器

配套代码实现文档：`docs/lesson_01_free_fall_code.md`

## 1. 物理假设

本讲只研究一个质点。质点是最小的动力学模型：

- 物体尺寸忽略不计。
- 不考虑转动。
- 质量集中在一个点上。
- 运动由位置、速度和加速度描述。

这不是说真实物体没有尺寸，而是当尺寸和转动对问题影响很小时，质点模型足够表达主要动力学。

## 2. 数学方程

Newton 第二定律：

$$
m\boldsymbol{a}=\boldsymbol{f}
$$

其中：

- 质量：

$$
m
$$

- 加速度：

$$
\boldsymbol{a}=\frac{d^2\boldsymbol{r}}{dt^2}
$$

- 外力合力：

$$
\boldsymbol{f}
$$

- 位置向量：

$$
\boldsymbol{r}=
\begin{bmatrix}
x \\
y \\
z
\end{bmatrix}
$$

这是二阶常微分方程：

$$
\frac{d^2\boldsymbol{r}}{dt^2}
=
\frac{1}{m}\boldsymbol{f}(\boldsymbol{r}, \boldsymbol{v}, t)
$$

数值求解器通常更喜欢一阶形式。定义状态：

$$
\boldsymbol{y}
=
\begin{bmatrix}
\boldsymbol{r} \\
\boldsymbol{v}
\end{bmatrix}
$$

则：

$$
\frac{d\boldsymbol{y}}{dt}
=
\begin{bmatrix}
\boldsymbol{v} \\
\frac{1}{m}\boldsymbol{f}(\boldsymbol{r}, \boldsymbol{v}, t)
\end{bmatrix}
$$

这一步非常关键：动力学仿真本质上是在随时间推进状态变量。

## 3. 物理本质

力不是直接改变位置，而是改变速度；速度再改变位置。动力学仿真的因果链是：

$$
\text{force}
\rightarrow
\text{acceleration}
\rightarrow
\text{velocity}
\rightarrow
\text{position}
$$

多体动力学后面会更复杂，但这个因果链不会消失。约束力、弹簧力、接触力、驱动力，最终都要进入加速度层。

## 4. 数学本质

把二阶方程改写成一阶状态空间，是为了得到标准初值问题：

$$
\begin{aligned}
\dot{\boldsymbol{y}} &= \boldsymbol{g}(t, \boldsymbol{y}), \\
\boldsymbol{y}(t_0) &= \boldsymbol{y}_0.
\end{aligned}
$$

一旦写成这个形式，Euler、Runge-Kutta、变步长积分器等数值方法都可以工作。

## 5. 工程应用

这个模型可以用于：

- 抛体运动
- 车辆或机器人质心的粗略平动模型
- 多体系统中每个刚体质心平动部分的基础
- 验证积分器、单位制、能量趋势和可视化工具

## 6. 本讲代码

相关代码：

- `mbd/integrators.py`
- `mbd/particles.py`
- `mbd/analysis.py`
- `examples/lesson_01_free_fall.py`

本讲示例模拟竖直方向自由落体，并检查机械能趋势。
