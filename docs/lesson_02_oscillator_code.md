# 第二讲代码实现：单自由度线性振子

理论推导见 `docs/lesson_02_generalized_coordinate_oscillator.md`。

运行命令：

```powershell
python -m examples.lesson_02_linear_oscillator
```

输出图像：`outputs/lesson_02_linear_oscillator.svg`

## 1. 文件职责

- `mbd/oscillators.py`：单自由度线性振子模型、固有频率和解析解；
- `mbd/integrators.py`：复用第一讲的固定步长 RK4；
- `mbd/analysis.py`：振子机械能和相对漂移；
- `mbd/plotting.py`：复用多面板 SVG 趋势图工具；
- `examples/lesson_02_linear_oscillator.py`：参数设置、仿真、收敛分析和输出入口。

这一章没有修改积分器。相同的积分器可以求解不同物理系统，前提是每个模型都提供统一形式的右端函数。

## 2. 状态向量与数组尺寸

理论状态为：

$$
\boldsymbol{y}
=
\begin{bmatrix}
q \\
\dot{q}
\end{bmatrix}
$$

代码中用长度为 2 的一维数组表示：

```python
y0 = np.array([q0, q_dot0], dtype=float)
```

对应关系为：

```text
y[0] -> q       -> 广义坐标，单位 m
y[1] -> q_dot   -> 广义速度，单位 m/s
```

积分结果 `history.y` 的形状是：

```text
(时间采样点数, 2)
```

所以全部时刻的位移和速度分别通过列索引获得：

```python
q = history.y[:, 0]
q_dot = history.y[:, 1]
```

## 3. 物理参数的数据模型

`LinearOscillator` 保存两个物理参数：

```python
@dataclass(frozen=True)
class LinearOscillator:
    mass: float
    stiffness: float
```

数学符号对应如下：

```text
mass      -> m -> 质量
stiffness -> k -> 刚度
```

`__post_init__` 在对象创建后检查参数：

```python
if self.mass <= 0.0:
    raise ValueError("mass must be positive")
```

负质量、零质量和非正刚度都不符合当前模型的物理假设，因此尽早报告错误比让无效参数进入积分器更可靠。

## 4. 从运动方程到右端函数

运动方程为：

$$
m\ddot{q}+kq=0
$$

解出加速度：

$$
\ddot{q}
=
-\frac{kq}{m}
$$

代码逐项对应：

```python
q, q_dot = y
q_ddot = -self.stiffness * q / self.mass
return np.array([q_dot, q_ddot], dtype=float)
```

返回数组就是：

$$
\dot{\boldsymbol{y}}
=
\begin{bmatrix}
\dot{q} \\
\ddot{q}
\end{bmatrix}
$$

### 4.1 序列解包

语句：

```python
q, q_dot = y
```

称为序列解包。它等价于：

```python
q = y[0]
q_dot = y[1]
```

当状态只有两个分量时，解包写法能够直接显示数学变量与数组分量的关系。如果数组长度不是 2，Python 会立即报错，也能帮助发现状态尺寸错误。

## 5. 固有频率属性

固有圆频率公式为：

$$
\omega_n=\sqrt{\frac{k}{m}}
$$

对应代码：

```python
@property
def natural_frequency(self) -> float:
    return float(np.sqrt(self.stiffness / self.mass))
```

`@property` 允许调用方像读取普通字段一样使用计算结果：

```python
omega_n = oscillator.natural_frequency
```

这里没有写圆括号，因为 `natural_frequency` 对外表现为只读属性，而不是需要调用方传参的操作。

## 6. 解析解如何代码化

位移解析解为：

$$
q(t)
=
q_0\cos(\omega_n t)
+
\frac{\dot{q}_0}{\omega_n}\sin(\omega_n t)
$$

速度解析解为：

$$
\dot{q}(t)
=
-q_0\omega_n\sin(\omega_n t)
+
\dot{q}_0\cos(\omega_n t)
$$

代码使用 NumPy 对整个时间数组一次计算：

```python
phase = omega_n * time
q = q0 * np.cos(phase) + (q_dot0 / omega_n) * np.sin(phase)
q_dot = -q0 * omega_n * np.sin(phase) + q_dot0 * np.cos(phase)
return np.column_stack((q, q_dot))
```

`np.column_stack` 把两条长度相同的一维数组按列组合：

```text
q      的形状: (N,)
q_dot  的形状: (N,)
返回值的形状: (N, 2)
```

这个形状与 `history.y` 完全一致，所以可以直接相减计算状态误差：

```python
state_error = np.linalg.norm(history.y - exact, axis=1)
```

`axis=1` 表示对每一个时间点的两个状态分量计算二范数。

## 7. 能量计算

总机械能为：

$$
E
=
\frac{1}{2}m\dot{q}^{2}
+
\frac{1}{2}kq^{2}
$$

代码对应：

```python
kinetic = 0.5 * mass * velocity * velocity
elastic = 0.5 * stiffness * displacement * displacement
return kinetic + elastic
```

这里的 `displacement` 和 `velocity` 都是长度为 \(N\) 的数组。NumPy 的乘法逐元素执行，因此一次得到全部时间点的能量。

## 8. 步长收敛分析

案例使用每周期 10、20、40 和 80 个时间步：

```python
steps_per_period = np.array([10, 20, 40, 80], dtype=int)
step_sizes = period / steps_per_period
```

每种步长都积分一个完整周期，并计算最终状态与解析状态的误差：

```python
errors[i] = np.linalg.norm(history.y[-1] - exact_final)
```

其中 `history.y[-1]` 使用负索引取得最后一行。

观测阶通过对数坐标中的直线拟合获得：

```python
observed_order = float(
    np.polyfit(np.log(step_sizes), np.log(errors), 1)[0]
)
```

对应数学关系：

$$
\log e
\approx
p\log(\Delta t)+\log C
$$

`np.polyfit(x, y, 1)` 拟合一次多项式，返回斜率和截距。索引 `[0]` 取得斜率，也就是观测收敛阶 \(p\)。

## 9. 四幅趋势图分别检查什么

### 9.1 位移时间历程

RK4 曲线与解析曲线应几乎重合。随着仿真时间增长，数值相位误差会逐渐积累，因此长时间比较比只看一个周期更严格。

### 9.2 相图

横轴是位移，纵轴是速度。无阻尼系统应形成闭合椭圆。如果轨迹逐圈扩大或缩小，说明数值算法在持续增加或耗散能量。

### 9.3 能量漂移

理论总能量恒定。数值能量漂移不应表现出显著的单调增长；减小步长后，漂移幅值应明显下降。

### 9.4 步长收敛图

横轴和纵轴分别是步长与最终误差的十进对数。RK4 的数据点应接近一条直线，拟合斜率应接近 4。

## 10. 完整数据流

```text
质量 m、刚度 k、初始状态 y0
-> 构造 LinearOscillator
-> 由 rhs() 生成状态方程右端
-> RK4 积分得到 history
-> 计算解析状态 exact
-> 比较状态误差与能量漂移
-> 用多组步长估计收敛阶
-> 输出位移、相图、能量和收敛趋势
```

这一数据流把“物理模型”和“通用积分器”分开。以后更换为摆、刚体或多自由度系统时，积分器接口仍然可以保持不变。
