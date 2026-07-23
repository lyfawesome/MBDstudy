# 第三讲代码实现：阻尼受迫线性振子

理论推导见 `docs/lesson_03_damping_forcing_resonance.md`。

运行命令：

```powershell
python -m examples.lesson_03_damped_forced_oscillator
```

输出图像：`outputs/lesson_03_damped_forced_oscillator.svg`

运行测试：

```powershell
python -m unittest discover -s tests
```

## 1. 本讲如何衔接前两讲

本讲继续使用三个已经建立的基础设施：

- `rhs(t, y) -> y_dot` 状态方程接口；
- `integrate_fixed_step` 和经典 RK4；
- `TimeHistory`、机械能分析与 SVG 绘图。

变化只发生在振子模型中。第二讲的模型为：

$$
m\ddot{q}+kq=0
$$

本讲扩展为：

$$
m\ddot{q}+c\dot{q}+kq=f(t)
$$

这说明积分器不需要理解阻尼和外力，物理模型负责计算状态导数。

## 2. 文件职责

- `mbd/oscillators.py`：振子参数、阻尼比、谐波外力、状态方程和稳态幅值；
- `mbd/analysis.py`：机械能与采样功率的累计梯形积分；
- `mbd/integrators.py`：复用固定步长 RK4；
- `mbd/plotting.py`：复用多面板 SVG 图；
- `examples/lesson_03_damped_forced_oscillator.py`：案例参数、仿真、验证和输出；
- `tests/test_oscillators.py`：模型方程、极限情况和分析函数的自动测试。

## 3. 在原有模型上增加可选参数

`LinearOscillator` 现在包含：

```python
@dataclass(frozen=True)
class LinearOscillator:
    mass: float
    stiffness: float
    damping: float = 0.0
    excitation: Excitation | None = None
```

对应关系为：

```text
mass       -> m    -> 质量
stiffness  -> k    -> 刚度
damping    -> c    -> 黏性阻尼系数
excitation -> f(t) -> 外部激励函数
```

默认值保持第二讲兼容：

```python
LinearOscillator(mass=2.0, stiffness=50.0)
```

等价于：

```python
LinearOscillator(
    mass=2.0,
    stiffness=50.0,
    damping=0.0,
    excitation=None,
)
```

因此第二讲无需修改，也不会因为新增功能改变原来的物理含义。

## 4. 外力为什么使用函数

外部激励的类型定义为：

```python
Excitation = Callable[[float], float]
```

它表示一个接收时间、返回标量力的函数：

```text
输入：t，单位 s
输出：f(t)，单位 N
```

谐波外力由工厂函数创建：

```python
excitation = harmonic_force(
    amplitude=1.5,
    angular_frequency=4.5,
)
```

内部返回的函数执行：

```python
force = amplitude * np.sin(angular_frequency * t + phase)
```

这种设计没有把正弦力写死在 `LinearOscillator` 中。以后可以传入阶跃力、脉冲近似、测量数据插值或控制器输出，而状态方程接口不变。

## 5. 右端函数逐项对应运动方程

模型中的核心代码为：

```python
q, q_dot = y
external_force = 0.0 if self.excitation is None else self.excitation(t)
q_ddot = (
    external_force - self.damping * q_dot - self.stiffness * q
) / self.mass
return np.array([q_dot, q_ddot], dtype=float)
```

它直接对应：

$$
\ddot{q}
=
\frac{f(t)-c\dot{q}-kq}{m}
$$

每次 RK4 调用 `rhs` 时都会传入新的 $t$、$q$ 和 $\dot{q}$，因此外力、阻尼力和弹簧力都会重新计算。代码没有预先规定响应频率或响应幅值。

## 6. 阻尼比属性

临界阻尼与阻尼比实现为只读属性：

```python
@property
def critical_damping(self) -> float:
    return float(2.0 * np.sqrt(self.mass * self.stiffness))

@property
def damping_ratio(self) -> float:
    return self.damping / self.critical_damping
```

案例先指定阻尼比，再反算阻尼系数：

```python
damping = damping_ratio * undamped.critical_damping
```

这样做便于比较不同质量和刚度系统的相对阻尼水平。

## 7. 稳态幅值计算

谐波稳态幅值为：

$$
A
=
\frac{F_0}
{\sqrt{(k-m\omega^2)^2+(c\omega)^2}}
$$

代码允许一次传入整个频率数组：

```python
frequency_ratio = np.linspace(0.1, 2.0, 400)
response_amplitude = oscillator.harmonic_steady_state_amplitude(
    force_amplitude,
    frequency_ratio * natural_frequency,
)
```

因此可以直接生成频率响应曲线，不需要编写 Python 循环。

如果无阻尼系统恰好在固有频率受到持续谐波激励，理论稳态幅值表达式的分母为零。代码将该点返回为 `np.inf`，表达理想线性模型中的无界响应趋势。

## 8. 能量平衡如何计算

首先计算每个采样点的机械能：

```python
energy = oscillator_mechanical_energy(mass, stiffness, q, q_dot)
```

外力功率和阻尼耗散功率分别为：

```python
input_power = force * q_dot
damping_power = damping * q_dot * q_dot
```

`cumulative_trapezoid` 对离散功率进行累计梯形积分：

```python
input_work = cumulative_trapezoid(input_power, history.t)
dissipated_energy = cumulative_trapezoid(damping_power, history.t)
```

随后比较：

```python
energy_change = energy - energy[0]
energy_balance = input_work - dissipated_energy
balance_residual = energy_change - energy_balance
```

理论上 `balance_residual` 应为零。实际计算中会留下时间积分和功率积分带来的小量数值误差。

## 9. 如何识别稳态幅值

案例运行 30 个激励周期，并只使用最后 5 个周期估计数值幅值：

```python
steady_mask = history.t >= history.t[-1] - 5.0 * forcing_period
numerical_amplitude = 0.5 * (
    np.max(q[steady_mask]) - np.min(q[steady_mask])
)
```

前面的数据包含正在衰减的瞬态响应，不能直接用于稳态幅值比较。末段的最大值与最小值之差的一半，是周期响应幅值的简单估计。

## 10. 四幅图分别检查什么

### 10.1 瞬态与稳态响应

数值响应最初与理论稳态曲线不同，因为初始条件引入了瞬态。随着时间增加，两条曲线应逐渐重合。

### 10.2 相轨迹

轨迹从初始状态逐渐靠近稳定的闭合环。最终闭合环对应外部激励维持的周期稳态，而不是机械能守恒。

### 10.3 能量平衡

`energy change` 与 `work - dissipation` 应几乎重合。它们明显分离通常意味着受力符号、功率表达式或时间积分存在错误。

### 10.4 频率响应

低频响应接近静力位移；接近固有频率时出现响应峰；高频时惯性占主导，位移幅值下降。

## 11. 自动测试覆盖什么

测试使用 Python 标准库 `unittest`，主要检查：

1. 默认参数仍产生第二讲的无阻尼方程；
2. `rhs` 正确包含弹簧力、阻尼力和外力；
3. 谐波响应满足零频静力极限和共振处的理论值；
4. 无阻尼解析解不会被错误用于有阻尼模型；
5. 非法阻尼和负激励频率会被拒绝；
6. 累计梯形积分能够精确积分线性采样数据。

这是项目第一次把理论极限和接口兼容性写成自动回归测试。

## 12. 完整数据流

```text
质量、刚度、阻尼比、谐波外力
-> 构造 LinearOscillator
-> rhs 在每次调用时计算外力、阻尼力和弹簧力
-> RK4 推进状态 [q, q_dot]
-> TimeHistory 保存完整时间历程
-> 计算机械能、外力做功和阻尼耗散
-> 提取末段数值稳态幅值
-> 与理论幅值及频率响应趋势比较
-> 输出时间历程、相图、能量平衡和频率响应
```

本讲仍然没有抽象通用多体系统。等双自由度模型出现后，再把标量质量、阻尼和刚度自然提升为矩阵，避免在只有一个真实用例时提前设计复杂接口。
