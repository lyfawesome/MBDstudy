# Multibody Dynamics Study Lab

这个仓库用于系统学习多体动力学及其编程求解。学习方式固定为：

1. 先明确物理假设。
2. 再写出数学方程。
3. 然后把方程转成可扩展的 Python 求解器代码。
4. 最后用可视化和趋势分析检查结果是否符合物理直觉。

当前起点：从质点 Newton 方程、状态变量、数值积分开始，逐步过渡到刚体运动学、约束方程、Lagrange 方程、DAE 求解和接触/摩擦。

## 目录

- `docs/roadmap.md`：长期学习路线。
- `docs/lesson_01_newton_particle.md`：第一讲，质点动力学与状态空间形式。
- `docs/lesson_01_free_fall_code.md`：第一讲案例代码实现说明。
- `docs/lesson_02_generalized_coordinate_oscillator.md`：第二讲，自由度、广义坐标与单自由度振子。
- `docs/lesson_02_oscillator_code.md`：第二讲振子案例的公式与代码对应。
- `mbd/`：逐步扩展的 Python 多体动力学求解器代码。
- `examples/`：每个概念对应的可运行示例。

## 本地环境

推荐使用 conda 创建学习环境：

```powershell
conda env create -f environment.yml
conda activate mbd-study
```

如果不使用 conda，也可以用 pip 安装依赖：

```powershell
pip install -r requirements.txt
```

## 运行第一讲示例

在仓库根目录运行：

```powershell
python -m examples.lesson_01_free_fall
```

示例会生成：

- `outputs/lesson_01_free_fall.svg`

## 运行第二讲示例

在仓库根目录运行：

```powershell
python -m examples.lesson_02_linear_oscillator
```

示例会生成：

- `outputs/lesson_02_linear_oscillator.svg`
