# 自由—自由模态结果进入 MBD 求解器：理论过程工作底稿

> 文档状态：第一版工作底稿，用于逐条提问、校正和补充实现细节。
>
> 当前范围：三维线性有限元柔性体、自由—自由模态、浮动坐标系、多体系统中的小弹性变形和大范围刚体运动。
>
> 暂不包含：Craig–Bampton 接口约束模态、材料非线性、几何大变形、接触面的模态恢复和应力恢复。

## 0. 如何使用这份工作底稿

本文把“自由模态结果如何进入 MBD”拆成以下链条：

1. 有限元模型给出质量矩阵、刚度矩阵、节点坐标和自由—自由模态；
2. 识别并删除六维刚体模态子空间；
3. 将保留的弹性模态转换到 MBD 柔性体浮动坐标系；
4. 计算与该模态基严格一致的降阶质量、刚度和阻尼矩阵；
5. MBD 求解器用整体平移、整体姿态和弹性模态坐标描述柔性体；
6. 求解器恢复节点和 marker 的位置、速度和加速度；
7. 求解器构造刚体—弹性耦合质量矩阵、弹性内力和外载荷广义力；
8. 柔性体与关节、载荷和其他物体一起装配到多体 DAE；
9. 时间积分后，再由模态基恢复节点运动。

后续讨论时可以直接引用章节名或文中的问题编号，例如“讨论 Q-FF-07”。

文中的结论分为三类：

- **确定**：可以由浮动坐标系和模态叠加理论直接得到；
- **当前采用**：为了形成完整推导而采用的建模约定；
- **待确认**：必须从 MBD 导入器源码、数据格式或开发者说明中确认。

---

## 1. 当前采用的基本假设

### 1.1 结构与变形假设

当前采用以下假设：

1. 柔性体材料为线弹性；
2. 柔性体相对于自身浮动坐标系发生小变形；
3. 柔性体浮动坐标系可以在全局空间中发生大平移和大转动；
4. 有限元质量矩阵和刚度矩阵在参考构型中为常量；
5. 模态基在柔性体浮动坐标系中为常量；
6. 当前只保留自由—自由弹性模态，不保留 C–B 接口物理坐标；
7. 三维自由体具有六维刚体模态子空间；
8. 浮动坐标系的整体运动已经描述这六个刚体自由度，因此弹性基中不再保留刚体模态。

### 1.2 坐标系约定

定义两个主要坐标系：

- 全局惯性坐标系 $G$；
- 柔性体浮动坐标系 $B$。

当前采用：

- 浮动坐标系原点位于柔性体未变形参考构型的质心；
- 旋转矩阵 $\mathbf A$ 将浮动坐标系 $B$ 中的向量转换到全局坐标系 $G$；
- 角速度 $\boldsymbol{\omega}$ 表达在浮动坐标系 $B$ 中；
- 因而旋转矩阵满足 $\dot{\mathbf A}=\mathbf A\widetilde{\boldsymbol{\omega}}$。

旋转、四元数顺序和角速度表达坐标系必须与实际 MBD 求解器一致。

### 1.3 符号与维数

设：

- $n$：有限元物理自由度总数；
- $N$：需要向 MBD 提供平移恢复数据的节点数；
- $m$：保留的自由弹性模态数；
- $\mathbf u_{\mathrm{FE}}\in\mathbb R^n$：有限元物理自由度位移；
- $\mathbf M_{\mathrm{FE}},\mathbf K_{\mathrm{FE}}\in\mathbb R^{n\times n}$：有限元质量、刚度矩阵；
- $\mathbf r\in\mathbb R^3$：浮动坐标系原点的全局位置；
- $\mathbf p\in\mathbb R^4$：柔性体姿态四元数；
- $\boldsymbol{\eta}\in\mathbb R^m$：弹性模态坐标；
- $\mathbf s_i\in\mathbb R^3$：节点 $i$ 在浮动坐标系中的参考位置；
- $\boldsymbol{\Psi}_i\in\mathbb R^{3\times m}$：节点 $i$ 的平移模态行块；
- $\boldsymbol{\Psi}\in\mathbb R^{3N\times m}$：所有节点平移模态行块组成的恢复矩阵。

对于任意向量 $\mathbf a\in\mathbb R^3$，定义反对称矩阵：

$$
\widetilde{\mathbf a}
=
\begin{bmatrix}
0&-a_z&a_y\\
a_z&0&-a_x\\
-a_y&a_x&0
\end{bmatrix}
$$

它满足：

$$
\widetilde{\mathbf a}\mathbf b
=
\mathbf a\times\mathbf b
$$

---

## 2. 第一步：有限元自由—自由特征值问题

有限元线性动力学方程为：

$$
\mathbf M_{\mathrm{FE}}\ddot{\mathbf u}_{\mathrm{FE}}
+
\mathbf C_{\mathrm{FE}}\dot{\mathbf u}_{\mathrm{FE}}
+
\mathbf K_{\mathrm{FE}}\mathbf u_{\mathrm{FE}}
=
\mathbf f_{\mathrm{FE}}
$$

自由—自由模态分析求解广义特征值问题：

$$
\boxed{
\mathbf K_{\mathrm{FE}}\boldsymbol{\phi}_j
=
\omega_j^2\mathbf M_{\mathrm{FE}}\boldsymbol{\phi}_j
}
$$

全部特征向量组成：

$$
\boldsymbol{\Phi}_{\mathrm{all}}
=
\begin{bmatrix}
\boldsymbol{\phi}_1&
\boldsymbol{\phi}_2&
\cdots&
\boldsymbol{\phi}_n
\end{bmatrix}
$$

理论上，三维自由体包含六个零特征值，对应三个整体平移和三个整体转动。实际数值计算中，这六个特征值通常只是接近零，并且求解器返回的六个向量可能是任意线性组合。

因此应将结果理解成两个子空间：

$$
\boldsymbol{\Phi}_{\mathrm{all}}
=
\begin{bmatrix}
\mathbf R&\boldsymbol{\Phi}_e
\end{bmatrix}
$$

其中：

- $\mathbf R\in\mathbb R^{n\times6}$ 为刚体模态子空间；
- $\boldsymbol{\Phi}_e\in\mathbb R^{n\times m}$ 为最终保留的弹性模态基。

对应的理想特征值结构为：

$$
\boldsymbol{\Lambda}
=
\begin{bmatrix}
\mathbf0_{6\times6}&\mathbf0\\
\mathbf0&\boldsymbol{\Omega}^2
\end{bmatrix}
$$

其中：

$$
\boldsymbol{\Omega}
=
\operatorname{diag}
\left(
\omega_1,\omega_2,\ldots,\omega_m
\right)
$$

这里的 $\omega_1$ 表示第一个弹性模态圆频率，不再包含刚体模态。

---

## 3. 第二步：构造、识别和删除六维刚体模态子空间

### 3.1 只有节点平移自由度时的解析刚体基

设节点 $i$ 相对于浮动坐标系原点的位置为 $\mathbf s_i$。整体发生微小平移 $\Delta\mathbf r$ 和微小转动 $\boldsymbol{\theta}$ 时，节点刚体位移为：

$$
\mathbf u_i^{\mathrm{rigid}}
=
\Delta\mathbf r
+
\boldsymbol{\theta}\times\mathbf s_i
$$

利用叉乘矩阵：

$$
\mathbf u_i^{\mathrm{rigid}}
=
\begin{bmatrix}
\mathbf I_3&-\widetilde{\mathbf s}_i
\end{bmatrix}
\begin{bmatrix}
\Delta\mathbf r\\
\boldsymbol{\theta}
\end{bmatrix}
$$

节点 $i$ 的解析刚体基为：

$$
\mathbf R_i
=
\begin{bmatrix}
\mathbf I_3&-\widetilde{\mathbf s}_i
\end{bmatrix}
\in\mathbb R^{3\times6}
$$

将所有节点堆叠：

$$
\mathbf R_t
=
\begin{bmatrix}
\mathbf R_1\\
\mathbf R_2\\
\vdots\\
\mathbf R_N
\end{bmatrix}
\in\mathbb R^{3N\times6}
$$

如果有限元物理自由度不止节点平移，还包括梁壳节点转角、内部自由度或 MPC 自由度，则必须在完整的 $n$ 维物理自由度空间中构造对应的 $\mathbf R\in\mathbb R^{n\times6}$，不能直接用 $\mathbf R_t$ 代替。

### 3.2 刚体子空间检查

解析刚体基应满足：

$$
\mathbf K_{\mathrm{FE}}\mathbf R
\approx
\mathbf0
$$

可以使用无量纲残差：

$$
\varepsilon_R
=
\frac{
\left\|
\mathbf K_{\mathrm{FE}}\mathbf R
\right\|_F
}{
\left\|
\mathbf K_{\mathrm{FE}}
\right\|_F
\left\|
\mathbf R
\right\|_F
}
$$

保留弹性模态应与刚体子空间质量正交：

$$
\boxed{
\mathbf R^T
\mathbf M_{\mathrm{FE}}
\boldsymbol{\Phi}_e
\approx
\mathbf0
}
$$

建议检查：

$$
\varepsilon_{R\Phi}
=
\frac{
\left\|
\mathbf R^T\mathbf M_{\mathrm{FE}}\boldsymbol{\Phi}_e
\right\|_F
}{
\left\|
\mathbf R^T\mathbf M_{\mathrm{FE}}\mathbf R
\right\|_F^{1/2}
\left\|
\boldsymbol{\Phi}_e^T\mathbf M_{\mathrm{FE}}\boldsymbol{\Phi}_e
\right\|_F^{1/2}
}
$$

### 3.3 去除弹性模态中的刚体污染

如果保留模态含有明显刚体分量，可构造质量内积下的投影矩阵：

$$
\mathbf P_{\perp R}
=
\mathbf I_n
-
\mathbf R
\left(
\mathbf R^T\mathbf M_{\mathrm{FE}}\mathbf R
\right)^{-1}
\mathbf R^T\mathbf M_{\mathrm{FE}}
$$

然后执行：

$$
\boldsymbol{\Phi}_e
\leftarrow
\mathbf P_{\perp R}\boldsymbol{\Phi}_e
$$

投影后必须重新进行质量正交化，并重新计算与新基一致的降阶矩阵。

### 3.4 为什么不能把六个刚体模态传成弹性模态

MBD 中，浮动坐标系已经通过 $\mathbf r$ 描述整体平移，通过 $\mathbf p$ 描述整体姿态。如果再把 $\mathbf R$ 的六列放入弹性基，就会让相同的刚体运动同时由两组坐标表示：

$$
\text{整体运动}
\quad\Longleftrightarrow\quad
\left\{
\mathbf r,\mathbf p
\right\}
\quad\text{和}\quad
\boldsymbol{\eta}_{\mathrm{rigid}}
$$

这会造成坐标冗余、质量矩阵病态或奇异，以及约束求解不稳定。

因此，当前方案传给 MBD 的弹性基必须是：

$$
\boxed{
\boldsymbol{\Phi}_{\mathrm{MBD}}
=
\boldsymbol{\Phi}_e
}
$$

而不是 $[\mathbf R\ \boldsymbol{\Phi}_e]$。

---

## 4. 第三步：定义 MBD 柔性体参考坐标系

### 4.1 参考系原点

当前取未变形参考构型的质心为浮动坐标系原点。设质心在 FE 坐标系中的位置为 $\mathbf c_0$。

如果采用集中节点质量：

$$
m
=
\sum_{i=1}^{N}m_i
$$

$$
\mathbf c_0
=
\frac{1}{m}
\sum_{i=1}^{N}
m_i\mathbf X_i^{\mathrm{FE}}
$$

对于一致质量矩阵、梁壳转动惯量、非结构质量和 MPC，质心与惯量应由有限元质量模型一致地计算，而不是简单使用几何节点平均。

### 4.2 参考系方向

设 $\mathbf Q\in\mathbb R^{3\times3}$ 将 MBD 浮动坐标系中的向量转换到 FE 坐标系。

节点在浮动坐标系中的参考坐标为：

$$
\boxed{
\mathbf s_i
=
\mathbf Q^T
\left(
\mathbf X_i^{\mathrm{FE}}-\mathbf c_0
\right)
}
$$

$\mathbf Q$ 可以选择为：

- 与 FE 全局坐标系一致；
- 与部件 CAD/装配局部坐标系一致；
- 与参考惯量主轴一致。

选择哪一种不是纯数学问题，它会影响节点坐标、模态分量、惯量矩阵、marker 坐标和输入文件可读性。所有数据必须使用同一个选择。

---

## 5. 第四步：提取并转换节点位移恢复基

### 5.1 从完整 FE 自由度提取节点平移

定义平移自由度提取矩阵：

$$
\mathbf E_t
\in
\mathbb R^{3N\times n}
$$

使得：

$$
\mathbf u_t
=
\mathbf E_t\mathbf u_{\mathrm{FE}}
$$

有限元弹性位移近似为：

$$
\mathbf u_{\mathrm{FE}}
\approx
\boldsymbol{\Phi}_e\boldsymbol{\eta}
$$

所以节点平移为：

$$
\mathbf u_t
\approx
\mathbf E_t\boldsymbol{\Phi}_e\boldsymbol{\eta}
$$

### 5.2 将模态分量旋转到浮动坐标系

定义块对角旋转矩阵：

$$
\mathbf Q_N^T
=
\mathbf I_N\otimes\mathbf Q^T
\in
\mathbb R^{3N\times3N}
$$

最终供节点运动恢复使用的模态矩阵为：

$$
\boxed{
\boldsymbol{\Psi}
=
\mathbf Q_N^T
\mathbf E_t
\boldsymbol{\Phi}_e
}
$$

其维数为：

$$
\boldsymbol{\Psi}
\in
\mathbb R^{3N\times m}
$$

节点 $i$ 对应的三行组成：

$$
\boldsymbol{\Psi}_i
=
\begin{bmatrix}
\psi_{ix,1}&\cdots&\psi_{ix,m}\\
\psi_{iy,1}&\cdots&\psi_{iy,m}\\
\psi_{iz,1}&\cdots&\psi_{iz,m}
\end{bmatrix}
\in
\mathbb R^{3\times m}
$$

节点弹性位移为：

$$
\boxed{
\mathbf u_{f,i}
=
\boldsymbol{\Psi}_i\boldsymbol{\eta}
}
$$

### 5.3 节点与自由度重排

如果 MBD 节点顺序与 FE 节点顺序不同，应定义节点置换矩阵 $\mathbf P_N$：

$$
\boldsymbol{\Psi}_{\mathrm{MBD}}
=
\mathbf P_N
\mathbf Q_N^T
\mathbf E_t
\boldsymbol{\Phi}_e
$$

最终节点坐标、节点质量、节点 ID 和模态三行块必须采用同一节点顺序。

---

## 6. 第五步：模态归一化和降阶矩阵

### 6.1 一般降阶矩阵

使用完整有限元弹性模态基计算：

$$
\boxed{
\begin{aligned}
\mathbf M_e
&=
\boldsymbol{\Phi}_e^T
\mathbf M_{\mathrm{FE}}
\boldsymbol{\Phi}_e,\\
\mathbf K_e
&=
\boldsymbol{\Phi}_e^T
\mathbf K_{\mathrm{FE}}
\boldsymbol{\Phi}_e,\\
\mathbf C_e
&=
\boldsymbol{\Phi}_e^T
\mathbf C_{\mathrm{FE}}
\boldsymbol{\Phi}_e
\end{aligned}
}
$$

其中：

$$
\mathbf M_e,\mathbf K_e,\mathbf C_e
\in
\mathbb R^{m\times m}
$$

### 6.2 质量归一化模态

如果模态按质量归一化：

$$
\boldsymbol{\Phi}_e^T
\mathbf M_{\mathrm{FE}}
\boldsymbol{\Phi}_e
=
\mathbf I_m
$$

则：

$$
\mathbf M_e=\mathbf I_m
$$

理想正交模态下：

$$
\mathbf K_e
=
\operatorname{diag}
\left(
\omega_1^2,\ldots,\omega_m^2
\right)
$$

若使用模态阻尼比 $\zeta_j$：

$$
\mathbf C_e
=
\operatorname{diag}
\left(
2\zeta_1\omega_1,
\ldots,
2\zeta_m\omega_m
\right)
$$

### 6.3 基变换必须同步作用于所有矩阵

如果为了正交化、重新排序或缩放而采用：

$$
\boldsymbol{\Phi}_{\mathrm{new}}
=
\boldsymbol{\Phi}_{\mathrm{old}}\mathbf S
$$

则必须同步变换：

$$
\boxed{
\begin{aligned}
\mathbf M_{e,\mathrm{new}}
&=
\mathbf S^T\mathbf M_{e,\mathrm{old}}\mathbf S,\\
\mathbf K_{e,\mathrm{new}}
&=
\mathbf S^T\mathbf K_{e,\mathrm{old}}\mathbf S,\\
\mathbf C_{e,\mathrm{new}}
&=
\mathbf S^T\mathbf C_{e,\mathrm{old}}\mathbf S
\end{aligned}
}
$$

只改变振型而不改变降阶矩阵，会改变结构的动能和势能。

---

## 7. 第六步：MBD 柔性体的状态变量

进入 MBD 后，求解器不再求解全部 $n$ 个有限元物理自由度。

位置状态为：

$$
\boxed{
\mathbf q
=
\begin{bmatrix}
\mathbf r\\
\mathbf p\\
\boldsymbol{\eta}
\end{bmatrix}
\in
\mathbb R^{7+m}
}
$$

其中：

- $\mathbf r\in\mathbb R^3$：浮动坐标系原点的全局位置；
- $\mathbf p\in\mathbb R^4$：姿态四元数；
- $\boldsymbol{\eta}\in\mathbb R^m$：弹性模态坐标。

四元数满足单位范数约束：

$$
\mathbf p^T\mathbf p=1
$$

定义速度变量：

$$
\boxed{
\boldsymbol{\nu}
=
\begin{bmatrix}
\dot{\mathbf r}\\
\boldsymbol{\omega}\\
\dot{\boldsymbol{\eta}}
\end{bmatrix}
\in
\mathbb R^{6+m}
}
$$

当前采用标量在前的四元数：

$$
\mathbf p
=
\begin{bmatrix}
p_0\\
\mathbf e
\end{bmatrix}
$$

旋转矩阵为：

$$
\mathbf A(\mathbf p)
=
\left(
p_0^2-\mathbf e^T\mathbf e
\right)\mathbf I_3
+
2\mathbf e\mathbf e^T
+
2p_0\widetilde{\mathbf e}
$$

四元数运动学关系为：

$$
\dot{\mathbf p}
=
\frac12
\mathbf E(\mathbf p)
\boldsymbol{\omega}
$$

其中：

$$
\mathbf E(\mathbf p)
=
\begin{bmatrix}
-\mathbf e^T\\
p_0\mathbf I_3+\widetilde{\mathbf e}
\end{bmatrix}
\in
\mathbb R^{4\times3}
$$

四元数分量顺序、旋转矩阵方向和角速度表达坐标系属于必须向开发者确认的接口约定。

---

## 8. 第七步：节点位置恢复

### 8.1 单节点公式

节点 $i$ 在当前浮动坐标系中的位置为：

$$
\boldsymbol{\rho}_i
=
\mathbf s_i
+
\boldsymbol{\Psi}_i\boldsymbol{\eta}
$$

节点全局位置为：

$$
\boxed{
\mathbf x_i
=
\mathbf r
+
\mathbf A(\mathbf p)
\left(
\mathbf s_i
+
\boldsymbol{\Psi}_i\boldsymbol{\eta}
\right)
}
$$

这个公式将三类信息组合起来：

- $\mathbf r$：柔性体整体平移；
- $\mathbf A$：柔性体整体转动；
- $\boldsymbol{\Psi}_i\boldsymbol{\eta}$：节点相对于浮动坐标系的弹性变形。

### 8.2 全部节点的矩阵形式

定义：

$$
\mathbf s
=
\begin{bmatrix}
\mathbf s_1\\
\vdots\\
\mathbf s_N
\end{bmatrix}
\in\mathbb R^{3N}
$$

$$
\mathbf x
=
\begin{bmatrix}
\mathbf x_1\\
\vdots\\
\mathbf x_N
\end{bmatrix}
\in\mathbb R^{3N}
$$

定义整体平移复制矩阵：

$$
\mathbf L
=
\mathbf1_N\otimes\mathbf I_3
\in
\mathbb R^{3N\times3}
$$

定义块对角姿态矩阵：

$$
\mathbf A_N
=
\mathbf I_N\otimes\mathbf A
\in
\mathbb R^{3N\times3N}
$$

全部节点位置为：

$$
\boxed{
\mathbf x
=
\mathbf L\mathbf r
+
\mathbf A_N
\left(
\mathbf s+\boldsymbol{\Psi}\boldsymbol{\eta}
\right)
}
$$

---

## 9. 第八步：节点速度和加速度

### 9.1 节点速度

对节点位置求导：

$$
\begin{aligned}
\dot{\mathbf x}_i
&=
\dot{\mathbf r}
+
\dot{\mathbf A}\boldsymbol{\rho}_i
+
\mathbf A\dot{\boldsymbol{\rho}}_i\\
&=
\dot{\mathbf r}
+
\mathbf A
\left(
\boldsymbol{\omega}\times\boldsymbol{\rho}_i
+
\boldsymbol{\Psi}_i\dot{\boldsymbol{\eta}}
\right)
\end{aligned}
$$

由于：

$$
\boldsymbol{\omega}\times\boldsymbol{\rho}_i
=
-\widetilde{\boldsymbol{\rho}}_i\boldsymbol{\omega}
$$

可以写成：

$$
\boxed{
\dot{\mathbf x}_i
=
\mathbf B_i\boldsymbol{\nu}
}
$$

其中节点速度雅可比为：

$$
\boxed{
\mathbf B_i
=
\begin{bmatrix}
\mathbf I_3&
-\mathbf A\widetilde{\boldsymbol{\rho}}_i&
\mathbf A\boldsymbol{\Psi}_i
\end{bmatrix}
}
$$

维数为：

$$
\mathbf B_i
\in
\mathbb R^{3\times(6+m)}
$$

### 9.2 节点加速度

再次求导：

$$
\boxed{
\begin{aligned}
\ddot{\mathbf x}_i
={}&
\ddot{\mathbf r}\\
&+
\mathbf A
\bigg[
\dot{\boldsymbol{\omega}}\times\boldsymbol{\rho}_i
+
\boldsymbol{\omega}\times
\left(
\boldsymbol{\omega}\times\boldsymbol{\rho}_i
\right)\\
&\qquad+
2\boldsymbol{\omega}\times
\boldsymbol{\Psi}_i\dot{\boldsymbol{\eta}}
+
\boldsymbol{\Psi}_i\ddot{\boldsymbol{\eta}}
\bigg]
\end{aligned}
}
$$

定义：

$$
\dot{\boldsymbol{\nu}}
=
\begin{bmatrix}
\ddot{\mathbf r}\\
\dot{\boldsymbol{\omega}}\\
\ddot{\boldsymbol{\eta}}
\end{bmatrix}
$$

则：

$$
\ddot{\mathbf x}_i
=
\mathbf B_i\dot{\boldsymbol{\nu}}
+
\mathbf a_{\mathrm{bias},i}
$$

偏置加速度为：

$$
\boxed{
\mathbf a_{\mathrm{bias},i}
=
\mathbf A
\left[
\boldsymbol{\omega}\times
\left(
\boldsymbol{\omega}\times\boldsymbol{\rho}_i
\right)
+
2\boldsymbol{\omega}\times
\boldsymbol{\Psi}_i\dot{\boldsymbol{\eta}}
\right]
}
$$

其中包含离心加速度和刚体转动—弹性速度的科氏加速度。

---

## 10. 第九步：柔性体耦合质量矩阵

本节先采用与开发者理论文档接近的集中节点质量模型。设节点 $i$ 的集中质量为 $m_i$。

动能为：

$$
T
=
\frac12
\sum_{i=1}^{N}
m_i
\dot{\mathbf x}_i^T\dot{\mathbf x}_i
$$

代入 $\dot{\mathbf x}_i=\mathbf B_i\boldsymbol{\nu}$：

$$
T
=
\frac12
\boldsymbol{\nu}^T
\left(
\sum_{i=1}^{N}
m_i\mathbf B_i^T\mathbf B_i
\right)
\boldsymbol{\nu}
$$

因此：

$$
\boxed{
\mathbf M_{\mathrm{FFRF}}
=
\sum_{i=1}^{N}
m_i\mathbf B_i^T\mathbf B_i
}
$$

定义总质量：

$$
m
=
\sum_{i=1}^{N}m_i
$$

定义当前一阶质量矩：

$$
\mathbf b(\boldsymbol{\eta})
=
\sum_{i=1}^{N}
m_i\boldsymbol{\rho}_i
$$

定义平动—模态耦合积分：

$$
\mathbf P
=
\sum_{i=1}^{N}
m_i\boldsymbol{\Psi}_i
\in\mathbb R^{3\times m}
$$

定义当前转动惯量：

$$
\mathbf J(\boldsymbol{\eta})
=
\sum_{i=1}^{N}
m_i
\widetilde{\boldsymbol{\rho}}_i^T
\widetilde{\boldsymbol{\rho}}_i
\in\mathbb R^{3\times3}
$$

定义转动—模态耦合积分：

$$
\mathbf H(\boldsymbol{\eta})
=
\sum_{i=1}^{N}
m_i
\widetilde{\boldsymbol{\rho}}_i
\boldsymbol{\Psi}_i
\in\mathbb R^{3\times m}
$$

对于纯节点平移集中质量模型：

$$
\mathbf M_e
=
\sum_{i=1}^{N}
m_i
\boldsymbol{\Psi}_i^T\boldsymbol{\Psi}_i
$$

展开得到：

$$
\boxed{
\mathbf M_{\mathrm{FFRF}}
=
\begin{bmatrix}
m\mathbf I_3
&
-\mathbf A\widetilde{\mathbf b}
&
\mathbf A\mathbf P
\\
\widetilde{\mathbf b}\mathbf A^T
&
\mathbf J
&
\mathbf H
\\
\mathbf P^T\mathbf A^T
&
\mathbf H^T
&
\mathbf M_e
\end{bmatrix}
}
$$

质量矩阵的三个坐标块依次对应：

$$
\begin{bmatrix}
\dot{\mathbf r}\\
\boldsymbol{\omega}\\
\dot{\boldsymbol{\eta}}
\end{bmatrix}
$$

### 10.1 自由模态正交性带来的简化

如果原点位于质心：

$$
\sum_i m_i\mathbf s_i=\mathbf0
$$

如果弹性模态与刚体平移模态质量正交：

$$
\mathbf P
=
\sum_i m_i\boldsymbol{\Psi}_i
\approx
\mathbf0
$$

如果弹性模态与刚体转动模态质量正交，则在未变形参考状态：

$$
\mathbf H(\mathbf0)
=
\sum_i
m_i
\widetilde{\mathbf s}_i
\boldsymbol{\Psi}_i
\approx
\mathbf0
$$

所以参考状态附近可能有：

$$
\mathbf M_{\mathrm{FFRF}}(\boldsymbol{\eta}=\mathbf0)
\approx
\begin{bmatrix}
m\mathbf I_3&\mathbf0&\mathbf0\\
\mathbf0&\mathbf J_0&\mathbf0\\
\mathbf0&\mathbf0&\mathbf M_e
\end{bmatrix}
$$

但 $\mathbf J(\boldsymbol{\eta})$ 和 $\mathbf H(\boldsymbol{\eta})$ 一般随弹性坐标变化，因此运动过程中仍可能出现非线性惯性耦合。

### 10.2 一致质量矩阵和节点转动惯量

如果有限元模型包含：

- 一致质量矩阵；
- 梁壳节点转动惯量；
- 刚性单元、MPC 或附加质量；
- 非对角节点质量耦合；

则上面的纯节点集中质量求和不一定与 $\mathbf M_{\mathrm{FE}}$ 完全等价。

此时应采用以下两种方案之一：

1. MBD 读取足够完整的有限元质量信息，并通过统一的运动学映射在线投影；
2. 模态程序预先计算 MBD 所需的全部惯性不变量和高阶耦合张量。

不能一方面使用一致质量有限元模态，另一方面又随意构造一组不等价的节点集中质量，而不检查动能误差。

---

## 11. 第十步：弹性势能、刚度和阻尼内力

在线性模态模型中，弹性势能为：

$$
V_e
=
\frac12
\boldsymbol{\eta}^T
\mathbf K_e
\boldsymbol{\eta}
$$

弹性刚度内力为：

$$
\mathbf f_{\mathrm{stiff},e}
=
\mathbf K_e\boldsymbol{\eta}
$$

阻尼内力为：

$$
\mathbf f_{\mathrm{damp},e}
=
\mathbf C_e\dot{\boldsymbol{\eta}}
$$

在完整的柔性体速度坐标中：

$$
\boxed{
\mathbf Q_{\mathrm{int}}
=
\begin{bmatrix}
\mathbf0_3\\
\mathbf0_3\\
\mathbf C_e\dot{\boldsymbol{\eta}}
+
\mathbf K_e\boldsymbol{\eta}
\end{bmatrix}
}
$$

刚度和阻尼只直接作用于弹性坐标，但通过质量耦合和约束仍会影响柔性体整体运动。

---

## 12. 第十一步：节点外力投影到 MBD 广义力

设节点 $i$ 受到表达在全局坐标系中的力 $\mathbf f_i^G$。

节点虚功为：

$$
\delta W_i
=
\left(\mathbf f_i^G\right)^T
\delta\mathbf x_i
$$

由于节点速度和虚位移使用相同的雅可比 $\mathbf B_i$，节点力产生的广义力为：

$$
\boxed{
\mathbf Q_i
=
\mathbf B_i^T\mathbf f_i^G
}
$$

展开：

$$
\boxed{
\begin{aligned}
\mathbf Q_{r,i}
&=
\mathbf f_i^G,\\
\mathbf Q_{\omega,i}
&=
\boldsymbol{\rho}_i
\times
\left(
\mathbf A^T\mathbf f_i^G
\right),\\
\mathbf Q_{\eta,i}
&=
\boldsymbol{\Psi}_i^T
\mathbf A^T\mathbf f_i^G
\end{aligned}
}
$$

对所有节点求和：

$$
\mathbf Q_{\mathrm{ext}}
=
\sum_{i=1}^{N}
\mathbf B_i^T\mathbf f_i^G
$$

模态广义力为：

$$
\boxed{
\mathbf Q_{\eta}
=
\boldsymbol{\Psi}^T
\mathbf A_N^T
\mathbf f^G
}
$$

这一步就是物理节点载荷进入模态坐标的载荷投影。

---

## 13. 第十二步：marker、关节与约束

### 13.1 marker 的平移恢复

设 marker $M$ 的参考局部位置为 $\mathbf s_M$，平移模态恢复矩阵为 $\boldsymbol{\Psi}_M\in\mathbb R^{3\times m}$。

marker 当前局部位置为：

$$
\boldsymbol{\rho}_M
=
\mathbf s_M
+
\boldsymbol{\Psi}_M\boldsymbol{\eta}
$$

marker 全局位置为：

$$
\boxed{
\mathbf x_M
=
\mathbf r
+
\mathbf A
\left(
\mathbf s_M
+
\boldsymbol{\Psi}_M\boldsymbol{\eta}
\right)
}
$$

marker 速度雅可比为：

$$
\boxed{
\mathbf J_M
=
\begin{bmatrix}
\mathbf I_3&
-\mathbf A\widetilde{\boldsymbol{\rho}}_M&
\mathbf A\boldsymbol{\Psi}_M
\end{bmatrix}
}
$$

因此：

$$
\dot{\mathbf x}_M
=
\mathbf J_M\boldsymbol{\nu}
$$

### 13.2 marker 平移约束

假设柔性体 marker 与另一个物体上的 marker 位置重合：

$$
\boldsymbol{\varphi}
=
\mathbf x_M-\mathbf x_B
=
\mathbf0
$$

约束反力由拉格朗日乘子 $\boldsymbol{\lambda}$ 表示。对柔性体产生的广义约束力为：

$$
\boxed{
\mathbf Q_{\mathrm{constraint}}
=
\boldsymbol{\varphi}_{\nu}^T
\boldsymbol{\lambda}
}
$$

由于约束雅可比包含 $\mathbf A\boldsymbol{\Psi}_M$，关节反力会进入弹性模态方程。这就是自由模态柔性体与 MBD 关节直接交互的核心。

### 13.3 marker 方向恢复

如果关节约束 marker 的方向，仅有节点三平移模态不一定足够。通常还需要定义：

$$
\boldsymbol{\Theta}_M
\in
\mathbb R^{3\times m}
$$

使小转角近似满足：

$$
\boldsymbol{\theta}_{f,M}
\approx
\boldsymbol{\Theta}_M\boldsymbol{\eta}
$$

$\boldsymbol{\Theta}_M$ 可以来自：

- 梁壳有限元节点转角；
- 刚性接口节点簇的最小二乘刚体拟合；
- 位移梯度或局部形函数；
- 有限元程序直接输出的转角恢复矩阵。

开发者理论文档主要明确了每节点三平移模态行块，但没有完全定义柔性 marker 的方向恢复接口，因此这是当前的重要待确认项。

---

## 14. 第十三步：完整柔性体动力学方程

利用节点偏置加速度，可以定义惯性偏置广义力：

$$
\mathbf h
=
\sum_{i=1}^{N}
\mathbf B_i^T
m_i
\mathbf a_{\mathrm{bias},i}
$$

它包含离心、科氏、陀螺以及随弹性坐标变化的惯性耦合项。

柔性体动力学方程可写成：

$$
\boxed{
\mathbf M_{\mathrm{FFRF}}
\dot{\boldsymbol{\nu}}
+
\mathbf h
+
\mathbf Q_{\mathrm{int}}
+
\boldsymbol{\varphi}_{\nu}^T
\boldsymbol{\lambda}
=
\mathbf Q_{\mathrm{ext}}
}
$$

在约束加速度层，求解器通常形成鞍点系统：

$$
\boxed{
\begin{bmatrix}
\mathbf M_{\mathrm{FFRF}}
&
\boldsymbol{\varphi}_{\nu}^T
\\
\boldsymbol{\varphi}_{\nu}
&
\mathbf0
\end{bmatrix}
\begin{bmatrix}
\dot{\boldsymbol{\nu}}\\
\boldsymbol{\lambda}
\end{bmatrix}
=
\begin{bmatrix}
\mathbf Q_{\mathrm{ext}}
-
\mathbf h
-
\mathbf Q_{\mathrm{int}}\\
\boldsymbol{\gamma}
\end{bmatrix}
}
$$

其中 $\boldsymbol{\gamma}$ 是位置约束、速度约束二次求导后得到的加速度约束右端项。

完整系统还应包含：

- 四元数运动学关系；
- 四元数单位范数约束或等价归一化处理；
- 其他刚体和柔性体的动力学方程；
- 所有关节和驱动约束；
- 时间积分算法的离散残差。

---

## 15. 第十四步：一个时间步内的数据使用顺序

在时刻 $t_n$，MBD 求解器已有：

$$
\mathbf r_n,
\mathbf p_n,
\boldsymbol{\eta}_n,
\dot{\mathbf r}_n,
\boldsymbol{\omega}_n,
\dot{\boldsymbol{\eta}}_n
$$

一个时间步或一次牛顿迭代中的主要过程如下。

### 步骤 1：计算姿态矩阵

$$
\mathbf A_n=\mathbf A(\mathbf p_n)
$$

### 步骤 2：恢复节点当前局部位置

$$
\boldsymbol{\rho}_{i,n}
=
\mathbf s_i
+
\boldsymbol{\Psi}_i\boldsymbol{\eta}_n
$$

### 步骤 3：恢复节点和 marker 全局位置

$$
\mathbf x_{i,n}
=
\mathbf r_n
+
\mathbf A_n\boldsymbol{\rho}_{i,n}
$$

### 步骤 4：计算节点速度雅可比

$$
\mathbf B_{i,n}
=
\begin{bmatrix}
\mathbf I_3&
-\mathbf A_n\widetilde{\boldsymbol{\rho}}_{i,n}&
\mathbf A_n\boldsymbol{\Psi}_i
\end{bmatrix}
$$

### 步骤 5：组装柔性体质量矩阵

$$
\mathbf M_n
=
\sum_i
m_i\mathbf B_{i,n}^T\mathbf B_{i,n}
$$

### 步骤 6：计算惯性偏置项

$$
\mathbf h_n
=
\sum_i
\mathbf B_{i,n}^T
m_i
\mathbf a_{\mathrm{bias},i,n}
$$

### 步骤 7：计算弹性内力

$$
\mathbf f_{e,n}
=
\mathbf C_e\dot{\boldsymbol{\eta}}_n
+
\mathbf K_e\boldsymbol{\eta}_n
$$

### 步骤 8：计算外力及其广义力

$$
\mathbf Q_{\mathrm{ext},n}
=
\sum_i
\mathbf B_{i,n}^T
\mathbf f_{i,n}^{G}
$$

### 步骤 9：计算约束和约束雅可比

$$
\boldsymbol{\varphi}_n,
\qquad
\boldsymbol{\varphi}_{\nu,n}
$$

### 步骤 10：求解动力学 DAE

求得：

$$
\ddot{\mathbf r}_{n+1},
\quad
\dot{\boldsymbol{\omega}}_{n+1},
\quad
\ddot{\boldsymbol{\eta}}_{n+1},
\quad
\boldsymbol{\lambda}_{n+1}
$$

### 步骤 11：时间积分并更新状态

更新：

$$
\mathbf r_{n+1},
\quad
\mathbf p_{n+1},
\quad
\boldsymbol{\eta}_{n+1}
$$

随后重新计算位置、质量矩阵、内力、外力和约束，直到当前时间步的非线性迭代收敛。

---

## 16. 从模态程序交付给 MBD 的数据包

### 16.1 坐标系和单位元数据

至少包括：

|数据|维数或类型|用途|
|---|---:|---|
|长度、质量、时间单位|字符串或枚举|保证量纲一致|
|FE 到 MBD 坐标旋转 $\mathbf Q$|$3\times3$|转换坐标和模态分量|
|参考质心 $\mathbf c_0$|$3\times1$|定义浮动坐标系原点|
|初始姿态约定|元数据|构造初始 $\mathbf A$ 或 $\mathbf p$|
|旋转正方向和四元数顺序|元数据|避免姿态约定错误|

### 16.2 节点运动恢复数据

至少包括：

|数据|维数|用途|
|---|---:|---|
|节点 ID|$N$ 个|节点映射|
|节点参考局部坐标 $\mathbf s_i$|每节点 $3\times1$|恢复节点位置|
|节点平移模态行块 $\boldsymbol{\Psi}_i$|每节点 $3\times m$|恢复弹性位移|
|节点顺序|元数据|保证坐标、质量和振型一致|
|自由度顺序|元数据|说明每组三行对应 $x,y,z$|

### 16.3 弹性降阶数据

至少包括：

|数据|维数|用途|
|---|---:|---|
|$\mathbf M_e$|$m\times m$|模态动能|
|$\mathbf K_e$|$m\times m$|弹性内力|
|$\mathbf C_e$ 或阻尼比|$m\times m$ 或 $m$ 个|阻尼内力|
|圆频率 $\omega_j$|$m$ 个|校验和阻尼定义|
|模态归一化说明|元数据|解释振型缩放|
|模态列顺序|元数据|保证矩阵与振型一致|

### 16.4 惯性数据：节点在线装配方案

如果 MBD 按节点在线组装浮动坐标质量矩阵，还应提供：

|数据|维数|用途|
|---|---:|---|
|节点质量 $m_i$|每节点一个|在线计算质量耦合|
|总质量 $m$|标量|一致性检查|
|参考惯量 $\mathbf J_0$|$3\times3$|一致性检查|
|质心一阶矩|$3\times1$|检查参考系原点|

如果有限元质量不能等价地集中到节点，还必须说明质量等效方法及动能误差。

### 16.5 惯性数据：预积分方案

如果 MBD 不读取节点质量，而读取预积分惯性数据，通常至少涉及：

$$
m,
\quad
\mathbf J_0,
\quad
\mathbf P,
\quad
\mathbf H_0
$$

如果求解器保留 $\mathbf J(\boldsymbol{\eta})$ 和 $\mathbf H(\boldsymbol{\eta})$ 对弹性坐标的完整依赖，还需要线性或二次惯性张量。具体张量定义和索引顺序不能仅从通用理论猜测，必须由导入格式或实现源码确认。

### 16.6 marker 和接口数据

对于每个关节、载荷或连接 marker，至少需要：

|数据|维数|用途|
|---|---:|---|
|marker 参考局部位置 $\mathbf s_M$|$3\times1$|恢复 marker 位置|
|marker 平移模态矩阵 $\boldsymbol{\Psi}_M$|$3\times m$|恢复 marker 弹性平移|
|marker 转角模态矩阵 $\boldsymbol{\Theta}_M$|$3\times m$，如需要|恢复 marker 弹性方向|
|marker 插值权重或节点簇|依实现而定|从节点恢复 marker|
|marker 局部坐标轴|$3\times3$|定义关节方向|

---

## 17. 只提供“频率和振型”为什么通常不够

如果只提供：

$$
\omega_j,
\qquad
\boldsymbol{\phi}_j
$$

MBD 仍可能缺少：

1. 振型的质量归一化方式；
2. 与振型严格一致的 $\mathbf M_e$、$\mathbf K_e$；
3. 节点参考坐标；
4. FE 坐标系到 MBD 浮动坐标系的转换；
5. 总质量、质心和惯量；
6. 节点质量或等价预积分惯性数据；
7. marker 平移和方向恢复矩阵；
8. 节点 ID 和自由度顺序；
9. 单位和旋转约定。

频率和模态振型只能描述“有哪些弹性形状和相对频率”，不能自动定义这些形状如何嵌入多体系统、如何承受关节反力以及如何与整体刚体运动耦合。

---

## 18. 自由模态直接耦合的适用性与局限

自由模态可以直接用于浮动坐标柔性体。关节力或节点力通过：

$$
\mathbf Q_{\eta}
=
\boldsymbol{\Psi}_M^T\mathbf f_M
$$

进入弹性模态方程。

但截断自由模态对局部静力柔度的表达可能不足，特别是：

- 载荷或关节集中作用在较小区域；
- 保留模态数量很少；
- 接口局部刚度和转角响应很重要；
- 需要准确恢复接口附近应力；
- 约束反力中含有较高空间频率成分。

此时可能需要：

- 增加自由模态数；
- 增加静力残余向量；
- 增加 attachment modes；
- 改用 Craig–Bampton 接口约束模态；
- 对 marker 区域建立刚性或分布式耦合。

自由模态方案适合作为第一阶段耦合验证，因为它没有 C–B 接口坐标，可以先验证最核心的浮动坐标运动学、模态内力、载荷投影和 DAE 装配。

---

## 19. 建议的最小验证算例

### 19.1 模态代数检查

- 检查 $\mathbf K_{\mathrm{FE}}\mathbf R\approx\mathbf0$；
- 检查 $\mathbf R^T\mathbf M_{\mathrm{FE}}\boldsymbol{\Phi}_e\approx\mathbf0$；
- 检查 $\boldsymbol{\Phi}_e^T\mathbf M_{\mathrm{FE}}\boldsymbol{\Phi}_e=\mathbf M_e$；
- 检查 $\boldsymbol{\Phi}_e^T\mathbf K_{\mathrm{FE}}\boldsymbol{\Phi}_e=\mathbf K_e$；
- 检查 $\omega_j^2$ 与 $\mathbf M_e^{-1}\mathbf K_e$ 的特征值一致。

### 19.2 坐标变换检查

- 随机选择节点，验证 FE 坐标变换到 MBD 坐标后的位置一致；
- 验证模态向量旋转前后的长度和物理位移一致；
- 验证节点、质量和模态行块的排序一致。

### 19.3 纯刚体运动检查

设置：

$$
\boldsymbol{\eta}=\mathbf0,
\qquad
\dot{\boldsymbol{\eta}}=\mathbf0
$$

柔性体应退化为刚体：

$$
\mathbf x_i
=
\mathbf r+\mathbf A\mathbf s_i
$$

不得出现无载荷弹性振动。

### 19.4 单模态自由振动检查

固定整体刚体运动，仅设置某一模态初始位移：

$$
\eta_k(0)=\eta_0
$$

无阻尼时应满足：

$$
\eta_k(t)
=
\eta_0\cos(\omega_k t)
$$

### 19.5 节点力投影检查

在节点或 marker 施加已知力，分别计算：

1. 物理空间虚功；
2. 模态空间虚功。

应满足：

$$
\delta\mathbf u^T\mathbf f
=
\delta\boldsymbol{\eta}^T\mathbf Q_{\eta}
$$

### 19.6 质量和能量检查

随机给定 $\dot{\mathbf r}$、$\boldsymbol{\omega}$ 和 $\dot{\boldsymbol{\eta}}$，比较：

$$
T_{\mathrm{node}}
=
\frac12\sum_i m_i\dot{\mathbf x}_i^T\dot{\mathbf x}_i
$$

与：

$$
T_{\mathrm{generalized}}
=
\frac12
\boldsymbol{\nu}^T
\mathbf M_{\mathrm{FFRF}}
\boldsymbol{\nu}
$$

二者应一致。

---

## 20. 当前必须向 MBD 开发者确认的问题

### Q-FF-01：柔性体导入架构

求解器采用哪一种方式？

1. 节点坐标、节点质量和节点模态行块在线组装；
2. 读取预积分惯性不变量；
3. 读取完整降阶矩阵并采用另一套惯性近似。

### Q-FF-02：自由度类型

求解器柔性节点只支持三个平移自由度，还是也支持梁壳节点转角？

### Q-FF-03：质量模型

节点质量是标量集中质量、每节点 $3\times3$ 质量块，还是来自一致质量矩阵的预积分结果？

### Q-FF-04：浮动坐标参考条件

求解器要求：

- 参考质心坐标系；
- 主惯量轴坐标系；
- mean-axis 条件；
- 与刚体模态质量正交；

其中哪些是强制要求？

### Q-FF-05：模态归一化

导入器期望质量归一化模态，还是同时读取一般的 $\mathbf M_e$ 和 $\mathbf K_e$？

### Q-FF-06：惯性非线性阶次

求解器是否保留 $\mathbf J(\boldsymbol{\eta})$、$\mathbf H(\boldsymbol{\eta})$ 随弹性坐标变化的项，还是只使用参考状态常量矩阵？

### Q-FF-07：四元数和角速度约定

- 四元数标量在前还是在后？
- $\mathbf A$ 是局部到全局还是全局到局部？
- $\boldsymbol{\omega}$ 表达在局部还是全局坐标系？
- 使用 $\dot{\mathbf A}=\mathbf A\widetilde{\boldsymbol{\omega}}$ 还是另一约定？

### Q-FF-08：marker 平移恢复

marker 必须对应实际节点，还是允许由多个节点加权插值得到？

### Q-FF-09：marker 方向恢复

六自由度关节所需的柔性 marker 方向和转角模态怎样定义？

### Q-FF-10：阻尼输入

求解器读取：

- 模态阻尼比 $\zeta_j$；
- Rayleigh 参数；
- 完整 $\mathbf C_e$；
- 其他阻尼模型；

中的哪一种？

### Q-FF-11：载荷和约束投影

节点力、marker 力、重力和关节反力是否统一通过节点/marker 雅可比投影到模态坐标？

### Q-FF-12：数据文件格式

需要明确字段名、矩阵存储顺序、索引起点、稀疏格式、浮点精度和版本信息。

### Q-FF-13：参考算例

开发者能否提供一个单柔性体自由—自由模态导入算例，包括期望的频率、静力位移和时域结果？

---

## 21. 当前理论结论

自由—自由有限元模态结果进入浮动坐标系 MBD 的核心关系是：

$$
\boxed{
\mathbf x_i(t)
=
\mathbf r(t)
+
\mathbf A\!\left(\mathbf p(t)\right)
\left[
\mathbf s_i
+
\boldsymbol{\Psi}_i\boldsymbol{\eta}(t)
\right]
}
$$

模态程序负责提供：

$$
\mathbf s_i,
\quad
\boldsymbol{\Psi}_i,
\quad
\mathbf M_e,
\quad
\mathbf K_e,
\quad
\mathbf C_e
$$

以及节点质量或等价惯性数据、坐标系定义和 marker 恢复数据。

MBD 求解器负责求解：

$$
\mathbf r(t),
\quad
\mathbf p(t),
\quad
\boldsymbol{\eta}(t)
$$

两者在以下位置发生真正的数学耦合：

1. 用 $\boldsymbol{\Psi}_i\boldsymbol{\eta}$ 恢复节点和 marker 变形；
2. 用节点速度雅可比构造刚体—弹性耦合质量矩阵；
3. 用 $\mathbf K_e\boldsymbol{\eta}$ 和 $\mathbf C_e\dot{\boldsymbol{\eta}}$ 计算弹性内力；
4. 用 $\boldsymbol{\Psi}_i^T\mathbf A^T\mathbf f_i^G$ 将节点力投影到模态坐标；
5. 用 marker 模态行块构造约束雅可比，使关节反力进入弹性方程；
6. 将整体运动、弹性运动和约束一起装配到多体 DAE 并进行时间积分。

---

## 22. 修订记录

### 版本 0.1

- 建立自由—自由模态进入浮动坐标系 MBD 的完整基础链条；
- 给出刚体模态识别和删除方法；
- 给出节点位置、速度、加速度恢复公式；
- 给出集中节点质量下的完整刚体—弹性耦合质量矩阵；
- 给出弹性内力、外力投影、marker 和约束关系；
- 列出当前必须向 MBD 开发者确认的问题。
