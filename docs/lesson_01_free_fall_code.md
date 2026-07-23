# 第一讲代码实现：自由落体质点案例

本文档只解释代码实现。理论推导见 `docs/lesson_01_newton_particle.md`。

本案例的目标是把质点 Newton 方程变成一套最小可运行的 Python 求解流程：

1. 用 `Particle` 表示一个质点。
2. 用 `constant_gravity` 表示重力。
3. 用 `rhs` 把二阶动力学方程转成一阶状态方程。
4. 用 `integrate_fixed_step` 和 `rk4_step` 推进时间。
5. 用能量趋势检查数值结果是否可信。
6. 用 `save_stacked_svg` 输出趋势图。

## 1. 文件对应关系

- `examples/lesson_01_free_fall.py`：案例入口，负责设置参数、运行仿真、输出图像。
- `mbd/particles.py`：质点模型和重力模型。
- `mbd/integrators.py`：固定步长积分器和 RK4 单步公式。
- `mbd/analysis.py`：机械能与相对漂移分析。
- `mbd/plotting.py`：简单 SVG 趋势图输出。

## 2. 数学对象到代码对象

### 2.1 状态变量

理论中的状态变量是：

$$
\boldsymbol{y}
=
\begin{bmatrix}
\boldsymbol{r} \\
\boldsymbol{v}
\end{bmatrix}
$$

其中位置和速度分别是：

$$
\boldsymbol{r}
=
\begin{bmatrix}
x \\
y \\
z
\end{bmatrix},
\qquad
\boldsymbol{v}
=
\begin{bmatrix}
v_x \\
v_y \\
v_z
\end{bmatrix}
$$

代码中用一个长度为 6 的 `numpy` 数组存储：

```python
initial_position = np.array([0.0, 0.0, 10.0])
initial_velocity = np.array([2.0, 0.0, 0.0])
y0 = np.concatenate((initial_position, initial_velocity))
```

对应关系是：

```text
y[0:3] -> position -> r
y[3:6] -> velocity -> v
```

这里 `np.concatenate` 的作用是把两个三维向量拼成一个六维状态向量。

### 2.2 动力学右端项

理论方程是：

$$
\frac{d\boldsymbol{y}}{dt}
=
\begin{bmatrix}
\boldsymbol{v} \\
\frac{1}{m}\boldsymbol{f}(\boldsymbol{r}, \boldsymbol{v}, t)
\end{bmatrix}
$$

在 `mbd/particles.py` 中对应为：

```python
def evaluate(t: float, y: Array) -> Array:
    position = y[:3]
    velocity = y[3:]
    acceleration = force(t, position, velocity) / self.mass
    return np.concatenate((velocity, acceleration))
```

逐行对应：

- `position = y[:3]` 取出位置向量。
- `velocity = y[3:]` 取出速度向量。
- `force(...) / self.mass` 对应加速度公式。
- `np.concatenate((velocity, acceleration))` 对应把速度和加速度拼成状态导数。

也就是：

$$
\dot{\boldsymbol{r}}=\boldsymbol{v}
$$

$$
\dot{\boldsymbol{v}}=\boldsymbol{a}
=
\frac{1}{m}\boldsymbol{f}
$$

## 3. 重力模型

本案例使用匀强重力场：

$$
\boldsymbol{f}_g
=
\begin{bmatrix}
0 \\
0 \\
-mg
\end{bmatrix}
$$

代码中对应：

```python
gravity_force = np.array([0.0, 0.0, -mass * g], dtype=float)
```

这里假设坐标系的 $z$ 轴向上，所以重力方向是负 $z$ 方向。

`constant_gravity` 返回的是一个函数：

```python
def force(_t: float, _position: Array, _velocity: Array) -> Array:
    return gravity_force
```

这个写法的含义是：虽然接口允许力依赖时间、位置和速度，但当前重力是常量，所以这些参数暂时不用。

参数名前面的下划线 `_t`、`_position`、`_velocity` 是 Python 中常见约定，表示“接口需要这个参数，但当前函数体不使用它”。

## 4. RK4 积分器

我们要求解的标准初值问题是：

$$
\dot{\boldsymbol{y}}
=
\boldsymbol{g}(t,\boldsymbol{y}),
\qquad
\boldsymbol{y}(t_0)=\boldsymbol{y}_0
$$

RK4 单步公式是：

$$
\begin{aligned}
\boldsymbol{k}_1 &= \boldsymbol{g}(t_n,\boldsymbol{y}_n), \\
\boldsymbol{k}_2 &= \boldsymbol{g}\left(t_n+\frac{\Delta t}{2},\boldsymbol{y}_n+\frac{\Delta t}{2}\boldsymbol{k}_1\right), \\
\boldsymbol{k}_3 &= \boldsymbol{g}\left(t_n+\frac{\Delta t}{2},\boldsymbol{y}_n+\frac{\Delta t}{2}\boldsymbol{k}_2\right), \\
\boldsymbol{k}_4 &= \boldsymbol{g}(t_n+\Delta t,\boldsymbol{y}_n+\Delta t\boldsymbol{k}_3), \\
\boldsymbol{y}_{n+1} &= \boldsymbol{y}_n+\frac{\Delta t}{6}\left(\boldsymbol{k}_1+2\boldsymbol{k}_2+2\boldsymbol{k}_3+\boldsymbol{k}_4\right).
\end{aligned}
$$

代码中对应 `mbd/integrators.py`：

```python
k1 = rhs(t, y)
k2 = rhs(t + 0.5 * dt, y + 0.5 * dt * k1)
k3 = rhs(t + 0.5 * dt, y + 0.5 * dt * k2)
k4 = rhs(t + dt, y + dt * k3)
return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
```

这里的 `rhs` 就是数学上的 $\boldsymbol{g}(t,\boldsymbol{y})$。

## 5. 时间历史数据结构

`TimeHistory` 用来保存积分结果：

```python
@dataclass(frozen=True)
class TimeHistory:
    t: Array
    y: Array
```

对应数学含义：

- `t[i]` 是第 $i$ 个时间点。
- `y[i]` 是第 $i$ 个时间点的状态向量。

所以：

```python
position = history.y[:, :3]
velocity = history.y[:, 3:]
```

含义是：

- `history.y[:, :3]` 取所有时间点的前三列，也就是所有位置。
- `history.y[:, 3:]` 取所有时间点的后三列，也就是所有速度。

这里的 `:` 是切片语法。二维数组 `history.y[:, :3]` 中，第一个 `:` 表示所有行，第二个 `:3` 表示第 0、1、2 列。

## 6. 能量趋势分析

自由落体在无空气阻力时机械能守恒：

$$
E
=
E_{\mathrm{kin}}
+
E_{\mathrm{pot}}
$$

动能：

$$
E_{\mathrm{kin}}
=
\frac{1}{2}m\boldsymbol{v}^{T}\boldsymbol{v}
$$

势能：

$$
E_{\mathrm{pot}}
=
mgz
$$

代码中对应：

```python
kinetic = 0.5 * mass * np.sum(velocity * velocity, axis=1)
potential = mass * g * position[:, 2]
return kinetic + potential
```

关键语法：

- `velocity * velocity` 是逐元素相乘。
- `np.sum(..., axis=1)` 表示对每一行求和，即计算每个时间点的速度平方和。
- `position[:, 2]` 取所有时间点的 $z$ 坐标。

相对漂移定义为：

$$
\delta_E(t)
=
\frac{E(t)-E(t_0)}{\max(|E(t_0)|,1)}
$$

代码中对应：

```python
reference = abs(float(values[0]))
scale = reference if reference > 1e-12 else 1.0
return (values - values[0]) / scale
```

当初始值明显非零时，代码使用初始值的绝对值作为尺度，得到真正的相对漂移；当初始值接近零时，改用 1.0 作为尺度，避免除以很小数导致数值放大。

## 7. 可视化数据流

案例最终输出三组曲线：

1. 高度随时间变化：`history.t` 与 `position[:, 2]`
2. 轨迹：`position[:, 0]` 与 `position[:, 2]`
3. 能量漂移：`history.t` 与 `relative_drift(energy)`

代码中每条曲线用 `Curve` 表示：

```python
Curve(history.t, position[:, 2], "z(t)", "#1f77b4")
```

`Curve` 是一个数据容器：

```python
@dataclass(frozen=True)
class Curve:
    x: Array
    y: Array
    label: str
    color: str
```

它不负责计算，只负责把绘图需要的数据组织在一起。

## 8. 不常见 Python 语法解释

### 8.1 `from __future__ import annotations`

这个语句让类型注解延迟求值。对本项目的好处是：以后类之间互相引用时，类型注解更不容易产生导入顺序问题。

### 8.2 `@dataclass(frozen=True)`

`dataclass` 自动生成初始化方法，让类更适合作为数据容器。

`frozen=True` 表示对象创建后字段不能随意修改。例如 `TimeHistory.t` 和 `TimeHistory.y` 更像一份仿真结果记录，而不是运行中不断被外部改写的对象。

### 8.3 `Callable[[float, Array], Array]`

这是类型注解，表示一个函数类型：

```text
输入：float 和 Array
输出：Array
```

在本项目里：

```python
Derivative = Callable[[float, Array], Array]
```

对应数学上的右端函数：

$$
\boldsymbol{g}(t,\boldsymbol{y})
$$

### 8.4 闭包函数

`Particle.rhs` 内部定义了 `evaluate`，然后把 `evaluate` 返回出去：

```python
def rhs(self, force: ForceLaw) -> Callable[[float, Array], Array]:
    def evaluate(t: float, y: Array) -> Array:
        ...
    return evaluate
```

这叫闭包。它的好处是：`evaluate` 可以记住 `self.mass` 和传入的 `force`，从而形成一个完整的动力学右端函数。

### 8.5 `if __name__ == "__main__"`

文件被直接运行时，`__name__` 等于 `"__main__"`，于是会执行：

```python
main()
```

文件被其他模块导入时，`main()` 不会自动运行。这样同一个文件既可以直接执行，也可以被测试或其他代码导入。

### 8.6 `sys.path.insert`

示例文件开头有：

```python
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
```

这是为了让下面两种运行方式都能找到 `mbd` 包：

```powershell
python examples\lesson_01_free_fall.py
```

```powershell
python -m examples.lesson_01_free_fall
```

长期更推荐第二种，因为它按 Python 包结构运行。

## 9. 本案例的数据流总结

完整数据流是：

```text
质量、重力、初始位置、初始速度
-> 构造初始状态 y0
-> 构造重力函数 force
-> 构造动力学右端 rhs
-> RK4 积分得到 TimeHistory
-> 从 history.y 中切片得到 position 和 velocity
-> 计算机械能和相对漂移
-> 输出 SVG 趋势图
```

这个流程就是以后多体动力学求解器的最小雏形。后续加入刚体、约束、关节、接触时，核心仍然是构造状态、构造方程右端、积分、验证趋势。
