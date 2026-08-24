# Craig–Bampton 降阶结果回传 MBD 求解器：理论、数据契约与验证指南

## 0. 本指南解决什么问题

本文基于开发者提供的 [`mutibody dynamics.md`](../Theory/mutibody%20dynamics.md) 全文及其配图，回答以下问题：

1. 该 MBD 求解器如何描述和使用柔性体；
2. Craig–Bampton（C–B）结果进入 MBD 后经历什么数学过程；
3. 你至少需要向求解器传递哪些数据；
4. 哪些字段能从理论文档确定，哪些仍必须通过导入器源码、输入样例或开发者确认；
5. 如何在交付前验证降阶数据不会因为坐标系、自由度顺序、归一化或接口节点定义而失效。

> **最重要的结论**：只传固有频率和降阶质量/刚度矩阵通常不够。该文档中的求解器采用浮动坐标系法，需要用节点参考坐标和模态振型恢复柔性体上任意点的位置、速度、约束和载荷；其质量矩阵还包含刚体平动、刚体转动与弹性模态之间的耦合。安全的交付包至少应包含参考坐标系定义、节点/接口映射、完整的位移基、降阶矩阵、模态频率与归一化说明。若求解器按文档中的集中质量公式在线组装惯性项，还必须提供节点质量，或提供等价的预积分惯性不变量。

### 0.1 当前协作边界

当前分工应明确为：

- 你负责从有限元模型生成可验证的 C–B 降阶结果，并把它转换成求解器约定的数据；
- MBD 求解器开发者负责定义并实现柔性体导入 schema，以及在求解器内部使用这些数据；
- 双方共同负责一个最小柔性体参考算例和跨程序回归测试。

因此，你不需要替合作者重写 MBD 求解器，但必须交付足以重建柔性体运动学、惯性、弹性、载荷和约束的数据。反过来，开发者不能只说“给我模态”，而应明确“模态基的定义、接口自由度表示和求解器实际读取的数据结构”。

### 0.2 针对你当前基础的讲解方式

从指定学习任务可以判断，你已经能够理解：

- 位置矢量分解、绝对速度与相对速度；
- 刚体固连矢量的导数；
- 动量矩、惯量张量以及张量在一组基下的矩阵表示；
- 同一物理对象与不同坐标表示之间的区别。

目前最需要补齐的不是再学一遍牛顿方程，而是把这些概念连接成下面这条链：

$$

\text{刚体固连矢量}
\longrightarrow
\text{可在连体系内变形的矢量}
\longrightarrow
\text{浮动坐标系柔性体}
\longrightarrow
\text{C--B 位移基}
\longrightarrow
\text{MBD 的质量、内力、载荷和约束}

$$

本文因此会反复注明：某个量在哪个坐标系中表示、求导相对于哪个参考系、哪些矩阵是常量、哪些量随姿态或弹性坐标变化。

### 0.3 现在最先要锁定的一个问题

在真正生成求解器文件前，必须由开发者确认柔性体导入器属于哪一种架构：

1. **节点在线装配型**：读取节点坐标、节点质量和节点模态行块，由求解器在线构造浮动坐标质量耦合；
2. **惯性不变量型**：读取预积分后的质量、惯量和模态耦合张量，由求解器运行时快速组合；
3. **完整降阶矩阵型**：直接读取某一套明确广义坐标下的完整质量、刚度和阻尼，但仍需节点或 marker 恢复基来处理连接和载荷。

理论文档最接近第一种，但不能仅凭理论文档断言真实导入器已经采用第一种。这个选择决定你最终发的是节点级数据、预积分张量，还是两者都发。

## 1. 对开发者理论文档的审读结论

### 1.1 文档的整体理论路线

全文的主线是：

1. 用质心平动和欧拉四元数描述刚体运动；
2. 用位置约束、速度约束和拉格朗日乘子组成受约束机械系统；
3. 对柔性体采用小变形假设和浮动坐标系法；
4. 柔性位移可以使用节点自由度，也可以使用模态基降阶；
5. 从动能和势能得到质量矩阵、刚度矩阵与广义力；
6. 把刚体、柔性体、约束和载荷装配为 DAE；
7. 使用 generalized-$\alpha$ 方法进行隐式积分和牛顿迭代。

原文还覆盖运动副、摩擦、弹簧、轮胎、气动力、接触、GEBT 梁和液压系统。这些章节说明了求解器的总体应用范围，但与你当前的 C–B 数据回传直接相关的内容主要是：

- “五、柔性体建模 / 5.1 模态叠加法”；
- “有限单元法 / 柔性体动能、势能、约束”；
- 第二处“柔性体建模 / 浮动坐标系法”；
- “DAE 方程的 Generalized-$\alpha$ 方法”；
- “程序开发过程中的一些实现考虑”。

### 1.2 文档能明确确认的事项

|事项|文档给出的信息|对数据交付的含义|
|---|---|---|
|运动描述|柔性体使用浮动坐标系法|必须定义柔性体参考坐标系及其初始位置/姿态|
|参考系原点|为简化计算，连体坐标系建在柔性体质心|节点坐标和振型必须表达在与求解器一致的质心坐标系中|
|变形假设|小变形、可叠加在线性模态基上|不适合用该接口直接表达局部大变形或材料强非线性|
|姿态参数|欧拉四元数|MBD 侧刚体姿态有 4 个坐标并附带单位范数约束|
|节点自由度|模态公式按每节点 $x,y,z$ 三个位移分量展开|文档明确支持的是三平移自由度；壳/梁节点转角的导入方式未说明|
|质量处理|模态章节采用集中质量法逐节点求和|可能需要节点集中质量及其顺序，而不仅是总质量|
|柔性位移|$\boldsymbol{u}_f=\boldsymbol{\Psi}\boldsymbol{c}$|求解器需要模态基在关心节点上的空间分量|
|质量耦合|存在平动–转动、平动–模态、转动–模态和模态–模态块|仅传对角频率不足以构造完整 FFRF 动力学|
|刚度|全阶柔性刚度由单元刚度装配|降阶后应提供或可计算 $\boldsymbol{K}_r$|
|约束点|柔性体约束位置包含节点柔性位移|接口节点振型值必须可用；关节不能只连接“刚性参考体”|
|时间积分|DAE 使用 generalized-$\alpha$ 和牛顿迭代|质量、内力、约束及其雅可比需要在每个迭代点保持一致|

### 1.3 文档没有给出的关键接口信息

以下内容无法从这份理论文档确定：

- 柔性体导入文件的格式、扩展名、版本号和字段名称；
- 数值采用文本、二进制、稠密矩阵还是稀疏矩阵；
- 节点编号从 0 还是从 1 开始（原文只说明求解器内部 body index 从 0 开始）；
- 模态列的最终顺序及其是否包含约束模态；
- 求解器期望原始 C–B 基，还是经过二次特征分解和质量正交化后的基；
- 求解器是读取 $\boldsymbol{M}_r,\boldsymbol{K}_r,\boldsymbol{C}_r$，还是根据节点质量和模态基自行计算；
- 模态阻尼的输入方式；
- 壳/梁节点的转动自由度如何处理；
- 接口节点、marker、主从节点和刚性连接区域的数据结构；
- 应力、应变和内力恢复矩阵是否受支持；
- 单位制、四元数分量顺序、矩阵存储顺序和浮点精度。

因此，本指南能够给出**数学上充分的数据契约**，但不能把某个自定义 JSON、HDF5、MNF 或二进制布局宣称为该求解器的真实文件协议。确认真实协议还需要至少一种额外证据：导入器源码、官方输入样例、schema，或开发者答复。

### 1.4 文档中的重要不完整点和潜在矛盾

1. 原文把

$$
   \boldsymbol{u}_f
   =
   \sum_{k=1}^{m}c_k(t)\boldsymbol{\Psi}_k

$$

   称为 Craig–Bampton，但这只是一般模态叠加形式。经典 C–B 必须明确包含**固定界面模态**和**静力约束模态**。

2. “程序实现”部分说全节点柔性体的坐标长度是 $8+3N$。这对应未降阶节点自由度加一个四元数约束乘子。使用 $m$ 个降阶坐标时，相应长度应是 $8+m$，而不是 $8+3N$。

3. 原文完整展开了柔性体动能，却没有完整写出模态降阶后的弹性势能、阻尼力、柔性体–刚体约束和柔性体载荷。

4. 原文的部分转动–模态质量耦合求和式缺少节点质量因子，与前面的单节点质量矩阵不一致。实现时应以动能积分或维度检查为准，不能机械照抄。

5. 主从节点位移在文档中用简单算术平均表示。这只适合非常有限的几何情形；真实刚性接口、RBE2/RBE3 或分布耦合通常需要带旋转一致性的插值权重。

## 2. 先建立正确的物理图景

### 2.1 柔性体的总运动不是“只有模态振动”

文档采用浮动坐标系法。柔性体上点 $P$ 的全局位置是：

$$

\boldsymbol{x}_P
=
\boldsymbol{r}
+
\boldsymbol{A}(\boldsymbol{p})
\left(
\bar{\boldsymbol{u}}_P
+
\boldsymbol{u}_{f,P}
\right)

$$

其中：

- $\boldsymbol{r}\in\mathbb{R}^3$：柔性体参考坐标系原点在全局坐标系的位置；
- $\boldsymbol{p}\in\mathbb{R}^4$：欧拉四元数；
- $\boldsymbol{A}(\boldsymbol{p})\in\mathbb{R}^{3\times3}$：从柔性体参考系到全局系的方向余弦矩阵；
- $\bar{\boldsymbol{u}}_P\in\mathbb{R}^3$：未变形点在柔性体参考系中的坐标；
- $\boldsymbol{u}_{f,P}\in\mathbb{R}^3$：该点相对参考系的弹性变形。

![浮动坐标系示意图](../Theory/FloatingFrame_FEM.png)

这意味着 MBD 中的柔性体同时具有：

- 3 个整体平移坐标；
- 4 个整体姿态参数和 1 个四元数归一化约束；
- $m$ 个弹性坐标。

如果使用 $m$ 个降阶坐标，则动力学坐标可写为：

$$

\boldsymbol{q}
=
\begin{bmatrix}
\boldsymbol{r}\\
\boldsymbol{p}\\
\boldsymbol{c}
\end{bmatrix}
\in\mathbb{R}^{7+m}

$$

加上四元数约束乘子后，单个柔性体对应的牛顿迭代块通常是 $8+m$ 阶。

### 2.2 模态基是几何映射，不只是频率表

把节点 $j$ 的模态行块记为：

$$

\boldsymbol{S}_j
=
\begin{bmatrix}
\Psi_{3j-2,1} & \cdots & \Psi_{3j-2,m}\\
\Psi_{3j-1,1} & \cdots & \Psi_{3j-1,m}\\
\Psi_{3j,1}   & \cdots & \Psi_{3j,m}
\end{bmatrix}
\in\mathbb{R}^{3\times m}

$$

则节点变形为：

$$

\boldsymbol{u}_{f,j}=\boldsymbol{S}_j\boldsymbol{c}

$$

把所有节点依次堆叠：

$$

\boldsymbol{u}_f
=
\boldsymbol{\Psi}\boldsymbol{c},
\qquad
\boldsymbol{\Psi}\in\mathbb{R}^{3N\times m}

$$

求解器使用 $\boldsymbol{\Psi}$ 完成至少四件事：

1. 从模态坐标恢复节点变形；
2. 构造刚体运动与弹性运动之间的质量耦合；
3. 把节点力投影为模态广义力；
4. 计算柔性节点处约束的残差和雅可比。

所以只有 $\omega_k$、$\boldsymbol{M}_r$ 和 $\boldsymbol{K}_r$ 而没有 $\boldsymbol{\Psi}$，无法在空间中定位关节、载荷或接触点。

### 2.3 从你已经掌握的刚体公式过渡到柔性体公式

对刚体上的固连点，连体系中的位置矢量 $\boldsymbol{s}$ 不随时间改变，因此：

$$

{}^{b}\dot{\boldsymbol{s}}=\boldsymbol{0}

$$

它在惯性系中的导数只来自连体系转动：

$$

{}^{I}\frac{d}{dt}
\left(
\boldsymbol{A}\boldsymbol{s}
\right)
=
\boldsymbol{A}
\left(
\boldsymbol{\omega}\times\boldsymbol{s}
\right)

$$

柔性体的关键区别是：点 $P$ 相对连体系的位置不是常量，而是

$$

\boldsymbol{s}_P
=
\bar{\boldsymbol{u}}_P
+
\boldsymbol{S}_P\boldsymbol{c}

$$

其中 $\bar{\boldsymbol{u}}_P$ 和 $\boldsymbol{S}_P$ 都固定在未变形参考构形中，是常量；$\boldsymbol{c}(t)$ 随时间变化。因此在连体系中：

$$

{}^{b}\dot{\boldsymbol{s}}_P
=
\boldsymbol{S}_P\dot{\boldsymbol{c}}

$$

对全局位置

$$

\boldsymbol{x}_P
=
\boldsymbol{r}
+
\boldsymbol{A}\boldsymbol{s}_P

$$

求导，必须同时保留“连体系转动”和“点在连体系中变形”两部分：

$$

\begin{aligned}
\dot{\boldsymbol{x}}_P
&=
\dot{\boldsymbol{r}}
+
\boldsymbol{A}
\left(
\boldsymbol{\omega}\times\boldsymbol{s}_P
+
{}^{b}\dot{\boldsymbol{s}}_P
\right) \\
&=
\dot{\boldsymbol{r}}
+
\boldsymbol{A}
\left[
\boldsymbol{\omega}\times
\left(
\bar{\boldsymbol{u}}_P
+
\boldsymbol{S}_P\boldsymbol{c}
\right)
+
\boldsymbol{S}_P\dot{\boldsymbol{c}}
\right]
\end{aligned}

$$

这正是开发者文档中柔性体速度公式的来源。与刚体相比，只多出一项：

$$

\boxed{
\boldsymbol{A}\boldsymbol{S}_P\dot{\boldsymbol{c}}
}

$$

它表示点相对浮动连体系的弹性速度。若令 $\boldsymbol{c}=\dot{\boldsymbol{c}}=\boldsymbol{0}$，公式立即退化为刚体点速度，这也是最基本的一致性检查。

### 2.4 速度雅可比为什么直接决定求解器需要哪些数据

利用反对称矩阵关系：

$$

\boldsymbol{\omega}\times\boldsymbol{s}_P
=
-\widetilde{\boldsymbol{s}}_P\boldsymbol{\omega}

$$

节点速度可以写成：

$$

\dot{\boldsymbol{x}}_P
=
\underbrace{
\begin{bmatrix}
\boldsymbol{I}
&
-\boldsymbol{A}\widetilde{\boldsymbol{s}}_P
&
\boldsymbol{A}\boldsymbol{S}_P
\end{bmatrix}
}_{\boldsymbol{B}_P}
\begin{bmatrix}
\dot{\boldsymbol{r}}\\
\boldsymbol{\omega}\\
\dot{\boldsymbol{c}}
\end{bmatrix}

$$

如果求解器用欧拉四元数 $\boldsymbol{p}$，并采用开发者文档中的关系：

$$

\boldsymbol{\omega}
=
2\boldsymbol{L}(\boldsymbol{p})\dot{\boldsymbol{p}}

$$

则速度雅可比变为：

$$

\boldsymbol{B}_P^{(p)}
=
\begin{bmatrix}
\boldsymbol{I}
&
-2\boldsymbol{A}\widetilde{\boldsymbol{s}}_P\boldsymbol{L}
&
\boldsymbol{A}\boldsymbol{S}_P
\end{bmatrix}

$$

这一个矩阵已经解释了数据需求：

- 计算 $\boldsymbol{s}_P$ 需要节点参考坐标 $\bar{\boldsymbol{u}}_P$ 和模态行块 $\boldsymbol{S}_P$；
- 计算弹性速度需要 $\boldsymbol{S}_P$；
- 计算整体惯性需要质量分布；
- 计算姿态耦合需要确定的连体系和四元数约定。

因此，“为什么要传节点坐标、模态振型和质量信息”不是文件格式偏好，而是由节点速度公式直接决定的。

## 3. 经典 Craig–Bampton 推导

### 3.1 自由度分块

将有限元自由度分成内部自由度 $i$ 和接口自由度 $b$：

$$

\boldsymbol{u}
=
\begin{bmatrix}
\boldsymbol{u}_b\\
\boldsymbol{u}_i
\end{bmatrix}

$$

全阶线性结构方程为：

$$

\begin{bmatrix}
\boldsymbol{M}_{bb} & \boldsymbol{M}_{bi}\\
\boldsymbol{M}_{ib} & \boldsymbol{M}_{ii}
\end{bmatrix}
\begin{bmatrix}
\ddot{\boldsymbol{u}}_b\\
\ddot{\boldsymbol{u}}_i
\end{bmatrix}
+
\begin{bmatrix}
\boldsymbol{K}_{bb} & \boldsymbol{K}_{bi}\\
\boldsymbol{K}_{ib} & \boldsymbol{K}_{ii}
\end{bmatrix}
\begin{bmatrix}
\boldsymbol{u}_b\\
\boldsymbol{u}_i
\end{bmatrix}
=
\begin{bmatrix}
\boldsymbol{f}_b\\
\boldsymbol{f}_i
\end{bmatrix}

$$

接口自由度必须覆盖 MBD 中施加关节、约束、集中载荷或其他力元的位置。标准工程实践也强调：CMS 生成时选择的接口节点应与后续 MBD 中的承载节点一致。

### 3.2 固定界面模态

令接口自由度固定：

$$

\boldsymbol{u}_b=\boldsymbol{0}

$$

内部自由度满足特征值问题：

$$

\boldsymbol{K}_{ii}\boldsymbol{\Phi}
=
\boldsymbol{M}_{ii}\boldsymbol{\Phi}\boldsymbol{\Omega}^2

$$

其中：

- $\boldsymbol{\Phi}\in\mathbb{R}^{n_i\times n_m}$ 是保留的固定界面振型；
- $\boldsymbol{\Omega}^2=\operatorname{diag}(\omega_1^2,\ldots,\omega_{n_m}^2)$。

它们描述接口固定时内部结构的动力变形。

### 3.3 静力约束模态

对每个接口自由度施加单位位移，忽略惯性并令其他接口自由度为零。内部静力平衡满足：

$$

\boldsymbol{K}_{ii}\boldsymbol{u}_i
+
\boldsymbol{K}_{ib}\boldsymbol{u}_b
=
\boldsymbol{0}

$$

因此约束模态矩阵为：

$$

\boldsymbol{\Psi}_c
=
-\boldsymbol{K}_{ii}^{-1}\boldsymbol{K}_{ib}

$$

它描述每一个接口单位位移在内部引起的静态变形。

### 3.4 C–B 变换

采用接口坐标 $\boldsymbol{q}_b$ 和固定界面模态坐标 $\boldsymbol{\eta}$：

$$

\begin{aligned}
\boldsymbol{u}_b
&=\boldsymbol{q}_b,\\
\boldsymbol{u}_i
&=\boldsymbol{\Psi}_c\boldsymbol{q}_b
+\boldsymbol{\Phi}\boldsymbol{\eta}
\end{aligned}

$$

写成矩阵形式：

$$

\boldsymbol{u}
=
\underbrace{
\begin{bmatrix}
\boldsymbol{I} & \boldsymbol{0}\\
\boldsymbol{\Psi}_c & \boldsymbol{\Phi}
\end{bmatrix}
}_{\boldsymbol{T}_{\mathrm{CB}}}
\underbrace{
\begin{bmatrix}
\boldsymbol{q}_b\\
\boldsymbol{\eta}
\end{bmatrix}
}_{\boldsymbol{q}_{\mathrm{CB}}}

$$

若你的全阶自由度顺序是 $[i,b]$，矩阵行块必须相应交换。自由度顺序不一致是最常见且最危险的导入错误之一。

### 3.5 降阶质量与刚度

$$

\boldsymbol{M}_{\mathrm{CB}}
=
\boldsymbol{T}_{\mathrm{CB}}^T
\boldsymbol{M}
\boldsymbol{T}_{\mathrm{CB}}

$$

$$

\boldsymbol{K}_{\mathrm{CB}}
=
\boldsymbol{T}_{\mathrm{CB}}^T
\boldsymbol{K}
\boldsymbol{T}_{\mathrm{CB}}

$$

在精确计算和一致分块下，刚度矩阵具有典型结构：

$$

\boldsymbol{K}_{\mathrm{CB}}
=
\begin{bmatrix}
\boldsymbol{K}_{bb}-\boldsymbol{K}_{bi}\boldsymbol{K}_{ii}^{-1}\boldsymbol{K}_{ib}
&
\boldsymbol{0}\\
\boldsymbol{0}
&
\boldsymbol{\Phi}^T\boldsymbol{K}_{ii}\boldsymbol{\Phi}
\end{bmatrix}

$$

但 $\boldsymbol{M}_{\mathrm{CB}}$ 一般不是对角阵。

### 3.6 为什么有些 MBD 文件中的模态已经“看不出”约束模态

常见柔性体生成流程还会对 C–B 基进行一次广义特征分解：

$$

\boldsymbol{K}_{\mathrm{CB}}\boldsymbol{N}
=
\boldsymbol{M}_{\mathrm{CB}}\boldsymbol{N}\boldsymbol{D}

$$

并定义最终基：

$$

\boldsymbol{Y}
=
\boldsymbol{T}_{\mathrm{CB}}\boldsymbol{N}

$$

若做了质量归一化，则：

$$

\boldsymbol{Y}^T\boldsymbol{M}\boldsymbol{Y}=\boldsymbol{I},
\qquad
\boldsymbol{Y}^T\boldsymbol{K}\boldsymbol{Y}=\boldsymbol{D}

$$

此时最终列向量是约束模态和固定界面模态的线性组合，接口行不再呈单位阵，但接口位移仍可通过 $\boldsymbol{Y}$ 的接口行恢复。开发者文档只写 $\boldsymbol{u}_f=\boldsymbol{\Psi}\boldsymbol{c}$，因此它既可能读取原始 C–B 基，也可能读取这种正交化后的最终基；必须确认导入器期望哪一种。

### 3.7 从你的 C–B 输出到求解器模态矩阵的变换链

你程序中的第一层关系是：

$$

\boldsymbol{u}_{\mathrm{FE}}
=
\boldsymbol{T}_{\mathrm{CB}}
\boldsymbol{q}_{\mathrm{CB}}

$$

若又进行一次正交化或广义特征分解：

$$

\boldsymbol{q}_{\mathrm{CB}}
=
\boldsymbol{N}\boldsymbol{c}

$$

则：

$$

\boldsymbol{u}_{\mathrm{FE}}
=
\underbrace{
\boldsymbol{T}_{\mathrm{CB}}
\boldsymbol{N}
}_{\boldsymbol{Y}}
\boldsymbol{c}

$$

若不做第二次变换，只需取 $\boldsymbol{N}=\boldsymbol{I}$。

开发者文档中的 $\boldsymbol{\Psi}$ 是按节点排列的三分量位移基，不一定等于你程序内部保存的 $\boldsymbol{T}_{\mathrm{CB}}$。更完整的转换关系是：

$$

\boxed{
\boldsymbol{\Psi}_{\mathrm{MBD}}
=
\boldsymbol{P}_{\mathrm{node}}
\left(
\boldsymbol{I}_N\otimes
\boldsymbol{R}_{\mathrm{FE}\rightarrow\mathrm{MBD}}
\right)
\boldsymbol{E}_{t}
\boldsymbol{T}_{\mathrm{CB}}
\boldsymbol{N}
}

$$

其中：

- $\boldsymbol{E}_{t}$ 从 FE 自由度中提取求解器需要的物理平移分量；
- $\boldsymbol{R}_{\mathrm{FE}\rightarrow\mathrm{MBD}}$ 把每个节点的三分量从 FE 坐标系旋转到柔性体连体系；
- $\boldsymbol{P}_{\mathrm{node}}$ 把 FE 自由度顺序重排为求解器节点顺序；
- $\boldsymbol{N}$ 记录原始 C–B 坐标到最终求解器弹性坐标的变换。

与最终基匹配的矩阵必须由同一变换得到：

$$

\begin{aligned}
\boldsymbol{M}_r
&=
\boldsymbol{N}^T
\boldsymbol{M}_{\mathrm{CB}}
\boldsymbol{N}, \\
\boldsymbol{K}_r
&=
\boldsymbol{N}^T
\boldsymbol{K}_{\mathrm{CB}}
\boldsymbol{N}, \\
\boldsymbol{C}_r
&=
\boldsymbol{N}^T
\boldsymbol{C}_{\mathrm{CB}}
\boldsymbol{N}
\end{aligned}

$$

因此必须成套交付：

$$

\left\{
\boldsymbol{\Psi}_{\mathrm{MBD}},
\boldsymbol{M}_r,
\boldsymbol{K}_r,
\boldsymbol{C}_r
\right\}

$$

不能把正交化后的 $\boldsymbol{\Psi}_{\mathrm{MBD}}$ 与正交化前的 $\boldsymbol{M}_{\mathrm{CB}}$、$\boldsymbol{K}_{\mathrm{CB}}$ 混用。

这里还有一个必须由开发者确认的接口问题：如果原始 C–B 接口包含转动自由度，$\boldsymbol{E}_t$ 不能简单删除这些转角行。必须先通过 FE 中实际使用的 RBE2、RBE3、MPC 或刚性接口映射，把接口平移和转动一致地转换为求解器可恢复的节点或 marker 运动。

## 4. C–B 结果进入该 MBD 求解器后的使用过程

### 4.1 运动恢复

对节点 $j$：

$$

\boldsymbol{x}_j
=
\boldsymbol{r}
+
\boldsymbol{A}
\left(
\bar{\boldsymbol{u}}_j
+
\boldsymbol{S}_j\boldsymbol{c}
\right)

$$

速度为：

$$

\dot{\boldsymbol{x}}_j
=
\dot{\boldsymbol{r}}
+
\boldsymbol{A}
\left[
\boldsymbol{\omega}
\times
\left(
\bar{\boldsymbol{u}}_j
+
\boldsymbol{S}_j\boldsymbol{c}
\right)
+
\boldsymbol{S}_j\dot{\boldsymbol{c}}
\right]

$$

因此模态坐标不仅决定变形，也进入节点速度和动能。

### 4.2 质量矩阵如何使用你的数据

对集中节点质量 $m_j$，定义：

$$

\boldsymbol{B}_j
=
\begin{bmatrix}
\boldsymbol{I}
&
-2\boldsymbol{A}\widetilde{\boldsymbol{u}}_j\boldsymbol{L}
&
\boldsymbol{A}\boldsymbol{S}_j
\end{bmatrix}

$$

其中：

$$

\boldsymbol{u}_j
=
\bar{\boldsymbol{u}}_j
+
\boldsymbol{S}_j\boldsymbol{c}

$$

节点对质量矩阵的贡献为：

$$

\boldsymbol{M}_j
=
m_j\boldsymbol{B}_j^T\boldsymbol{B}_j

$$

为了看清每一块从哪里来，先用角速度 $\boldsymbol{\omega}$ 而不是四元数速度。令：

$$

\boldsymbol{s}_j
=
\bar{\boldsymbol{u}}_j
+
\boldsymbol{S}_j\boldsymbol{c}

$$

节点速度雅可比为：

$$

\boldsymbol{B}_j^{(\omega)}
=
\begin{bmatrix}
\boldsymbol{I}
&
-\boldsymbol{A}\widetilde{\boldsymbol{s}}_j
&
\boldsymbol{A}\boldsymbol{S}_j
\end{bmatrix}

$$

把 $m_j{\boldsymbol{B}_j^{(\omega)}}^T\boldsymbol{B}_j^{(\omega)}$ 展开，可得到：

$$

\begin{aligned}
\boldsymbol{M}_{RR}
&=
\sum_jm_j\boldsymbol{I}, \\
\boldsymbol{M}_{R\omega}
&=
-\boldsymbol{A}
\sum_jm_j\widetilde{\boldsymbol{s}}_j, \\
\boldsymbol{M}_{Rc}
&=
\boldsymbol{A}
\sum_jm_j\boldsymbol{S}_j, \\
\boldsymbol{M}_{\omega\omega}
&=
\sum_jm_j
\widetilde{\boldsymbol{s}}_j^T
\widetilde{\boldsymbol{s}}_j, \\
\boldsymbol{M}_{\omega c}
&=
\sum_jm_j
\widetilde{\boldsymbol{s}}_j
\boldsymbol{S}_j, \\
\boldsymbol{M}_{cc}
&=
\sum_jm_j
\boldsymbol{S}_j^T\boldsymbol{S}_j
\end{aligned}

$$

其余下三角块由质量矩阵对称性得到。再用

$$

\boldsymbol{\omega}
=
2\boldsymbol{L}\dot{\boldsymbol{p}}

$$

把角速度坐标替换成四元数速度，即可得到开发者文档中带 $2\boldsymbol{L}$ 和 $4\boldsymbol{L}^T(\cdot)\boldsymbol{L}$ 的块。

这一步揭示了三个非常重要的事实：

1. $\boldsymbol{M}_{\omega\omega}$ 随 $\boldsymbol{c}$ 变化，因为 $\boldsymbol{s}_j$ 含有弹性变形；
2. $\boldsymbol{M}_{\omega c}$ 把整体转动与弹性速度耦合起来；
3. $\boldsymbol{M}_{cc}$ 就是该基下的降阶质量矩阵。

如果使用一致质量矩阵而不是集中节点质量，最后一项应写为：

$$

\boldsymbol{M}_{cc}
=
\boldsymbol{\Psi}^T
\boldsymbol{M}_{\mathrm{FE}}
\boldsymbol{\Psi}

$$

其他刚柔耦合块也必须由一致质量积分或等价惯性不变量得到，不能只拿总质量和总转动惯量代替。因此需要开发者明确导入器到底按集中质量节点求和，还是读取预积分不变量。

装配后得到：

$$

\boldsymbol{M}
=
\begin{bmatrix}
\boldsymbol{M}_{RR}
&
\boldsymbol{M}_{R\theta}
&
\boldsymbol{M}_{Rf}\\
\boldsymbol{M}_{\theta R}
&
\boldsymbol{M}_{\theta\theta}
&
\boldsymbol{M}_{\theta f}\\
\boldsymbol{M}_{fR}
&
\boldsymbol{M}_{f\theta}
&
\boldsymbol{M}_{ff}
\end{bmatrix}

$$

几个最容易理解的块是：

$$

\boldsymbol{M}_{RR}
=
\left(\sum_j m_j\right)\boldsymbol{I}

$$

$$

\boldsymbol{M}_{Rf}
=
\boldsymbol{A}
\sum_j m_j\boldsymbol{S}_j

$$

$$

\boldsymbol{M}_{ff}
=
\sum_j m_j\boldsymbol{S}_j^T\boldsymbol{S}_j
=
\boldsymbol{\Psi}^T\boldsymbol{M}_{\mathrm{FE}}\boldsymbol{\Psi}

$$

若参考系原点位于质心且模态满足适当的质量正交条件，部分耦合项可为零或很小；但不能未经验证就删除。

### 4.3 弹性内力

在线性降阶模型中：

$$

\boldsymbol{f}_{\mathrm{int},f}
=
\boldsymbol{K}_r\boldsymbol{c}
+
\boldsymbol{C}_r\dot{\boldsymbol{c}}

$$

若采用质量归一化正交模态，则常见形式为：

$$

\boldsymbol{M}_r=\boldsymbol{I},
\qquad
\boldsymbol{K}_r=\operatorname{diag}(\omega_k^2)

$$

按模态阻尼比 $\zeta_k$ 输入时：

$$

\boldsymbol{C}_r
=
\operatorname{diag}(2\zeta_k\omega_k)

$$

如果模态不是质量归一化，不能直接使用上式，必须按实际广义模态质量构造阻尼。

### 4.4 节点力如何变成模态广义力

若节点物理力在柔性体局部坐标系中堆叠为 $\boldsymbol{f}$，虚功等价给出：

$$

\delta W
=
\delta\boldsymbol{u}_f^T\boldsymbol{f}
=
\delta\boldsymbol{c}^T
\boldsymbol{\Psi}^T\boldsymbol{f}

$$

所以模态广义力为：

$$

\boldsymbol{Q}_f
=
\boldsymbol{\Psi}^T\boldsymbol{f}

$$

若全局力 $\boldsymbol{F}$ 作用在节点 $j$，先转换到柔性体局部坐标系：

$$

\boldsymbol{Q}_{f,j}
=
\boldsymbol{S}_j^T\boldsymbol{A}^T\boldsymbol{F}

$$

这再次说明：载荷点处的模态行块不可缺少。

### 4.5 关节和约束如何使用接口节点振型

假设柔性体接口点 $P$ 与另一物体上的点 $Q$ 重合：

$$

\boldsymbol{\Phi}
=
\boldsymbol{x}_P-\boldsymbol{x}_Q
=
\boldsymbol{0}

$$

柔性体接口点位置含有：

$$

\boldsymbol{x}_P
=
\boldsymbol{r}
+
\boldsymbol{A}
\left(
\bar{\boldsymbol{u}}_P
+
\boldsymbol{S}_P\boldsymbol{c}
\right)

$$

约束对模态坐标的雅可比包含：

$$

\frac{\partial\boldsymbol{x}_P}{\partial\boldsymbol{c}}
=
\boldsymbol{A}\boldsymbol{S}_P

$$

因此接口节点或 marker 的坐标、插值权重和模态振型必须精确对应。若关节连接在一个刚性化接口面上，还必须明确从接口面节点到 marker 的运动学映射，不能只给出一个几何中心坐标。

### 4.6 进入 DAE 与时间积分

求解器最终求解的形式可概括为：

$$

\begin{aligned}
\boldsymbol{M}(\boldsymbol{q})\ddot{\boldsymbol{q}}
+
\boldsymbol{h}(\boldsymbol{q},\dot{\boldsymbol{q}})
+
\boldsymbol{f}_{\mathrm{int}}(\boldsymbol{c},\dot{\boldsymbol{c}})
+
\boldsymbol{\Phi}_{\boldsymbol{q}}^T\boldsymbol{\lambda}
&=
\boldsymbol{Q}_{\mathrm{ext}},\\
\boldsymbol{\Phi}(\boldsymbol{q},t)
&=\boldsymbol{0}
\end{aligned}

$$

其中：

- $\boldsymbol{M}(\boldsymbol{q})$ 来自节点质量、节点坐标和模态基；
- $\boldsymbol{h}$ 是由配置相关质量矩阵产生的科氏、离心和其他二次速度惯性项；
- 弹性部分的 $\boldsymbol{f}_{\mathrm{int}}$ 含有 $\boldsymbol{K}_r\boldsymbol{c}+\boldsymbol{C}_r\dot{\boldsymbol{c}}$；
- $\boldsymbol{Q}_{\mathrm{ext}}$ 中的模态分量由 $\boldsymbol{\Psi}^T$ 投影得到；
- $\boldsymbol{\Phi}_{\boldsymbol{q}}$ 的模态列由接口处 $\boldsymbol{A}\boldsymbol{S}_P$ 形成；
- $\boldsymbol{\lambda}$ 是约束反力对应的拉格朗日乘子。

因此你的数据在最终方程中的位置可以概括为：

$$

\boxed{
\left\{
\bar{\boldsymbol{u}}_j,
m_j,
\boldsymbol{S}_j,
\boldsymbol{K}_r,
\boldsymbol{C}_r,
\text{interface mapping}
\right\}
\longrightarrow
\left\{
\boldsymbol{M},
\boldsymbol{h},
\boldsymbol{f}_{\mathrm{int}},
\boldsymbol{Q}_{\mathrm{ext}},
\boldsymbol{\Phi}_{\boldsymbol{q}}
\right\}
}

$$

在每个 generalized-$\alpha$ 时间步中，求解器会预测 $\boldsymbol{q}$、$\dot{\boldsymbol{q}}$、$\ddot{\boldsymbol{q}}$，然后重复：

1. 用当前 $\boldsymbol{c}$ 更新节点变形；
2. 更新配置相关的质量矩阵和惯性项；
3. 计算弹性内力、外力和约束力；
4. 计算约束残差及雅可比；
5. 解牛顿修正方程直至收敛。

你的数据不是一次性用于“初始化模态”，而是在整个瞬态求解中反复参与运动恢复、惯性、力和约束计算。

## 5. 建议向求解器交付的数据

### 5.1 最小安全数据契约

|数据组|字段或数组|典型维度|必要性|用途|
|---|---|---:|---|---|
|版本与单位|格式版本、长度/质量/时间/角度单位|标量/字符串|必需|防止量纲错误|
|参考系|原点、轴方向、右手性、初始姿态|$3$、$3\times3$ 或四元数|必需|把 FE 数据映射到 MBD 连体坐标系|
|刚体属性|总质量、质心、参考点惯量张量|$1,3,3\times3$|必需或可重建|整体惯性与校核|
|节点表|节点 ID、参考坐标、自由度顺序|$N\times(1+3)$|必需|运动恢复、接口与可视化|
|节点质量|每节点集中质量，或一致质量矩阵|$N$ 或 $3N\times3N$|按文档大概率必需|构造 FFRF 质量和惯性耦合|
|接口定义|接口节点/marker ID、激活分量、插值权重|按接口数|必需|关节、载荷和约束定位|
|位移基|最终传给 MBD 的 $\boldsymbol{\Psi}$|$3N\times m$|必需|节点变形、力投影、约束雅可比|
|降阶质量|$\boldsymbol{M}_r$|$m\times m$|必需或可重建|模态惯性|
|降阶刚度|$\boldsymbol{K}_r$|$m\times m$|必需|弹性恢复力|
|阻尼|$\boldsymbol{C}_r$ 或 $\zeta_k$|$m\times m$ 或 $m$|通常必需|耗能和数值稳定性|
|模态信息|$\lambda_k=\omega_k^2$、$f_k$、类型、顺序|$m$|强烈建议|诊断、阻尼和一致性检查|
|基的定义|原始 C–B / 正交化 C–B、归一化、符号与列顺序|元数据|必需|正确解释矩阵和坐标|
|拓扑|用于显示/接触的单元连接关系|按网格|按功能需要|可视化、表面和接触|
|恢复矩阵|应力/应变/内力恢复算子|依实现|按功能需要|柔性应力后处理|

### 5.2 两种可能的导入架构

#### 架构 A：求解器从节点数据在线构造 FFRF 惯性项

这与开发者文档的逐节点集中质量求和最接近。需要传递：

- $\bar{\boldsymbol{u}}_j$：节点参考坐标；
- $m_j$：节点集中质量；
- $\boldsymbol{S}_j$：每个节点的最终模态行块；
- $\boldsymbol{K}_r$、阻尼信息；
- 接口和 marker 映射。

优点是求解器可以从统一数据重算所有配置相关的惯性块。缺点是文件较大，而且节点质量和模态自由度顺序必须完全一致。

#### 架构 B：求解器读取预计算的惯性不变量

某些成熟 MBD 柔性体格式会预先保存质量、惯量和模态积分量，运行时用这些不变量快速生成质量矩阵。至少可能包括：

- 总质量 $m$；
- 质心和参考惯量 $\boldsymbol{J}_0$；
- 一阶质量矩 $\sum_j m_j\bar{\boldsymbol{u}}_j$；
- 平动–模态积分 $\sum_jm_j\boldsymbol{S}_j$；
- 转动–模态积分 $\sum_jm_j\widetilde{\bar{\boldsymbol{u}}}_j\boldsymbol{S}_j$；
- 模态质量 $\sum_jm_j\boldsymbol{S}_j^T\boldsymbol{S}_j$；
- 由模态变形引起的惯量一阶、二阶张量；
- $\boldsymbol{K}_r$、$\boldsymbol{C}_r$；
- 接口节点处模态行块。

如果导入器使用这种架构，只给节点数据却不给预积分张量也可能无法读取。必须以真实 schema 为准。

### 5.3 你目前的 C–B 结果是否足够

可以按下面的判断表快速检查：

|你现在已有的结果|是否足够|缺什么|
|---|---|---|
|只有固有频率|不够|模态基、质量/刚度、节点和接口映射|
|频率 + 固定界面振型|不够|静力约束模态；这还不是完整 C–B|
|$\boldsymbol{T}_{\mathrm{CB}}$ + $\boldsymbol{M}_{\mathrm{CB}}$ + $\boldsymbol{K}_{\mathrm{CB}}$|数学降阶基本完整，但通常仍不够导入 MBD|节点坐标、接口定义、参考系、阻尼及求解器格式|
|最终正交基 $\boldsymbol{\Psi}$ + $\boldsymbol{M}_r$ + $\boldsymbol{K}_r$|接近可交付|仍需节点/接口映射、质量或惯性不变量、元数据|
|上述全部 + 节点质量 + 参考坐标 + 接口/marker|数学上通常充分|仅剩求解器真实文件布局和可选恢复数据|

### 5.4 你现在应准备的交付包 v0.1

在开发者尚未冻结真实 schema 时，建议先准备一个不损失信息的中间数据包。它分为“运行必需数据”和“验收追溯数据”。

#### A. 运行必需数据

1. `metadata`：版本、单位、浮点精度、矩阵存储顺序、索引起点；
2. `reference_frame`：FE 到 MBD 的旋转、原点、右手性、参考构形定义；
3. `rigid_properties`：总质量、质心、参考点惯量；
4. `nodes`：节点 ID、三维参考坐标、求解器节点顺序；
5. `mass_representation`：节点集中质量，或开发者指定的惯性不变量；
6. `interfaces`：接口 ID、节点集合、marker 位置与方向、RBE/MPC/插值映射；
7. `basis`：最终 $\boldsymbol{\Psi}_{\mathrm{MBD}}\in\mathbb{R}^{3N\times m}$；
8. `reduced_matrices`：与该基严格一致的 $\boldsymbol{M}_r$、$\boldsymbol{K}_r$、$\boldsymbol{C}_r$；
9. `modal_metadata`：特征值、圆频率、Hz 频率、模态类型、归一化、列顺序；
10. `topology_or_recovery`：显示、接触或应力恢复需要的可选数据。

#### B. 验收与追溯数据

1. 内部自由度和接口自由度清单；
2. 固定界面模态 $\boldsymbol{\Phi}$；
3. 静力约束模态 $\boldsymbol{\Psi}_c$；
4. $\boldsymbol{T}_{\mathrm{CB}}$、$\boldsymbol{M}_{\mathrm{CB}}$、$\boldsymbol{K}_{\mathrm{CB}}$；
5. 二次变换矩阵 $\boldsymbol{N}$；
6. FE 质量、刚度来源文件的校验和；
7. 静力残差、特征残差、对称性和正定性报告；
8. 一个可视化模态文件和一个最小回归算例。

验收数据未必全部进入求解器运行文件，但没有它们，双方很难定位“导入后频率不对”究竟来自 C–B 计算、坐标转换、矩阵错配还是导入器解析。

### 5.5 发给开发者的最简结论

可以把需求压缩成下面一句话：

> 我将提供同一参考系、同一自由度顺序和同一归一化下的节点参考坐标、质量表示、接口映射、最终节点位移基、降阶质量/刚度/阻尼和频率元数据；请你确认导入器读取原始 C–B 坐标还是正交化坐标，读取节点质量还是预积分惯性不变量，以及接口转动和 marker 的表示方法。

## 6. 接口节点是整个交付中最需要谨慎的部分

### 6.1 哪些节点必须作为接口节点

至少包括：

- 与刚体或其他柔性体建立运动副的节点；
- 弹簧、阻尼器、衬套、执行器等力元的作用点；
- 集中载荷、驱动或传感器所在点；
- 可能参与接触且求解器要求作为保留自由度的节点；
- 刚性连接区域对应的独立参考节点或 marker。

若某个 MBD 承载点在 C–B 计算时没有被当成接口，它的局部静态柔度可能被截断，导致连接点过硬、反力错误或高频响应失真。

### 6.2 接口自由度必须一致

开发者文档的模态矩阵只显式包含每节点 3 个平移分量。若你的 FE 模型使用壳、梁、刚性单元或 6 自由度接口，必须确认：

1. 求解器是否支持接口转角；
2. 是否先用刚性区域把接口的 6 自由度运动转换成一组平移节点运动；
3. 求解器是否需要独立 marker 的位置和方向模态；
4. 转动自由度在降阶和导出时是否已经凝聚。

在这一点没有答案之前，不应直接把 $6N\times m$ 振型矩阵截取为 $3N\times m$。简单删除转角行会改变接口运动学和能量一致性。

### 6.3 主从节点不是简单平均就一定正确

开发者文档给出了从节点位移的算术平均示意：

![主从节点示意图](../Theory/MasterAndSlaveNodes.png)

更一般的接口映射应写成：

$$

\boldsymbol{u}_{\mathrm{marker}}
=
\boldsymbol{H}_{\mathrm{interface}}\boldsymbol{u}_{\mathrm{interface}}

$$

从而 marker 的模态基为：

$$

\boldsymbol{\Psi}_{\mathrm{marker}}
=
\boldsymbol{H}_{\mathrm{interface}}
\boldsymbol{\Psi}_{\mathrm{interface}}

$$

$\boldsymbol{H}_{\mathrm{interface}}$ 应与 FE 中使用的 RBE2、RBE3、MPC 或面积加权耦合一致，并能表达刚性平移与转动。

## 7. 坐标系、单位和归一化

### 7.1 推荐的参考系约定

开发者文档建议把柔性体连体坐标系建在质心。交付前建议完成：

1. 把未变形节点坐标平移到质心；
2. 明确轴方向和右手性；
3. 记录 FE 全局系到 MBD 连体系的旋转矩阵 $\boldsymbol{R}_{\mathrm{FE}\rightarrow\mathrm{MBD}}$；
4. 对每个节点坐标和每个模态的三分量应用同一个旋转；
5. 把惯量张量按同一旋转变换。

坐标和振型变换为：

$$

\bar{\boldsymbol{u}}_j^{\mathrm{MBD}}
=
\boldsymbol{R}_{\mathrm{FE}\rightarrow\mathrm{MBD}}
\left(
\bar{\boldsymbol{u}}_j^{\mathrm{FE}}-\boldsymbol{x}_{\mathrm{COM}}^{\mathrm{FE}}
\right)

$$

$$

\boldsymbol{S}_j^{\mathrm{MBD}}
=
\boldsymbol{R}_{\mathrm{FE}\rightarrow\mathrm{MBD}}
\boldsymbol{S}_j^{\mathrm{FE}}

$$

惯量变换为：

$$

\boldsymbol{J}^{\mathrm{MBD}}
=
\boldsymbol{R}_{\mathrm{FE}\rightarrow\mathrm{MBD}}
\boldsymbol{J}^{\mathrm{FE}}
\boldsymbol{R}_{\mathrm{FE}\rightarrow\mathrm{MBD}}^T

$$

### 7.2 单位换算必须同时作用于所有相关数据

若长度缩放为 $s_L$、质量缩放为 $s_M$、时间缩放为 $s_T$，则：

- 节点坐标和位移型模态按 $s_L$ 缩放；
- 质量按 $s_M$ 缩放；
- 转动惯量按 $s_Ms_L^2$ 缩放；
- 刚度按 $s_M/s_T^2$ 缩放；
- 频率按 $1/s_T$ 缩放。

但模态缩放还取决于归一化。若把模态列乘以任意常数，模态坐标、降阶质量和刚度必须做一致的逆变换，不能单独缩放振型图。

### 7.3 模态归一化的等价变换

若旧基与新基满足：

$$

\boldsymbol{\Psi}_{\mathrm{new}}
=
\boldsymbol{\Psi}_{\mathrm{old}}\boldsymbol{S}

$$

为保持物理位移不变：

$$

\boldsymbol{c}_{\mathrm{old}}
=
\boldsymbol{S}\boldsymbol{c}_{\mathrm{new}}

$$

降阶矩阵必须同步变换：

$$

\boldsymbol{M}_{\mathrm{new}}
=
\boldsymbol{S}^T\boldsymbol{M}_{\mathrm{old}}\boldsymbol{S}

$$

$$

\boldsymbol{K}_{\mathrm{new}}
=
\boldsymbol{S}^T\boldsymbol{K}_{\mathrm{old}}\boldsymbol{S}

$$

$$

\boldsymbol{C}_{\mathrm{new}}
=
\boldsymbol{S}^T\boldsymbol{C}_{\mathrm{old}}\boldsymbol{S}

$$

只替换振型而不变换矩阵，会改变系统的物理能量。

### 7.4 防止整体刚体运动被重复表示

浮动坐标系已经用 $\boldsymbol{r}$ 和 $\boldsymbol{p}$ 描述整体平动与转动。如果最终位移基中又保留相同的刚体分量，广义坐标可能出现重复描述、强耦合甚至奇异。

令 $\boldsymbol{R}_{\mathrm{rigid}}$ 为 FE 节点自由度上的六个刚体运动基。常用的质量正交条件是：

$$
\boldsymbol{R}_{\mathrm{rigid}}^T
\boldsymbol{M}_{\mathrm{FE}}
\boldsymbol{\Psi}
\approx
\boldsymbol{0}
$$

在质心连体系和集中质量表示下，它对应：

$$
\sum_jm_j\boldsymbol{S}_j
\approx
\boldsymbol{0}
$$

以及：

$$
\sum_jm_j
\widetilde{\bar{\boldsymbol{u}}}_j
\boldsymbol{S}_j
\approx
\boldsymbol{0}
$$

第一式消除模态中的整体平移质量分量，第二式消除模态中的整体转动质量分量。它们会使参考构形处的一些刚柔质量耦合块消失。

但这不是所有 FFRF 实现都强制采用的唯一坐标条件。有些求解器允许这些耦合非零，并在完整质量矩阵中保留它们。因此不能自行删除刚柔耦合块；必须确认导入器是否要求：

- 已剔除六个刚体模态的弹性基；
- mean-axis 或其他参考条件；
- 与刚体模态质量正交的最终基；
- 或允许任意基并显式读取全部耦合不变量。

## 8. 数据维度与顺序建议

### 8.1 建议的自由度顺序声明

必须在元数据中显式声明，例如：

```text
node_order      = [101, 102, 205, ...]
nodal_dof_order = [ux, uy, uz]
modal_order     = [mode_1, mode_2, ..., mode_m]
matrix_storage  = row_major | column_major
index_base      = 0 | 1
```

如果原始 FE 自由度顺序为按分量排列，例如全部 $u_x$、全部 $u_y$、全部 $u_z$，而求解器按节点交错排列，就必须用置换矩阵 $\boldsymbol{P}$：

$$

\boldsymbol{\Psi}_{\mathrm{solver}}
=
\boldsymbol{P}\boldsymbol{\Psi}_{\mathrm{FE}}

$$

### 8.2 推荐的逻辑数据结构

下面只是与求解器无关的逻辑 schema，用来核对内容完整性，不代表真实文件格式：

```yaml
format:
  name: flexible_body_exchange
  version: 1
units:
  length: m
  mass: kg
  time: s
reference_frame:
  origin_definition: center_of_mass
  handedness: right
  rotation_fe_to_mbd: [[...], [...], [...]]
rigid_properties:
  mass: ...
  center_of_mass: [0, 0, 0]
  inertia_about_reference: [[...], [...], [...]]
nodes:
  ids: [...]
  coordinates: [[x1, y1, z1], ...]
  lumped_masses: [...]
interfaces:
  - id: joint_A
    node_ids: [...]
    active_components: [ux, uy, uz]
    marker_position: [...]
    interpolation: {...}
reduction:
  method: craig_bampton
  basis_variant: raw_cb | orthogonalized_cb
  normalization: mass | maximum_component | none
  basis: Psi
  reduced_mass: Mr
  reduced_stiffness: Kr
  reduced_damping: Cr
  eigenvalues: [...]
  frequencies_hz: [...]
recovery:
  element_connectivity: optional
  stress_recovery: optional
```

## 9. 交付前的数值验证

### 9.1 基础结构检查

- $\boldsymbol{M}_r$、$\boldsymbol{K}_r$、$\boldsymbol{C}_r$ 的维度均与模态列数一致；
- $\boldsymbol{\Psi}$ 的行数与节点自由度数一致；
- 节点 ID 唯一，接口节点都能在节点表中找到；
- 所有数组无 NaN、Inf 和未初始化值；
- 质量、刚度矩阵对称误差满足：

$$
  \frac{\|\boldsymbol{M}_r-\boldsymbol{M}_r^T\|_F}
  {\|\boldsymbol{M}_r\|_F}
  \ll 1

$$

$$
  \frac{\|\boldsymbol{K}_r-\boldsymbol{K}_r^T\|_F}
  {\|\boldsymbol{K}_r\|_F}
  \ll 1

$$

### 9.2 C–B 特有检查

1. 约束模态满足静力残差：

$$
   \boldsymbol{K}_{ii}\boldsymbol{\Psi}_c
   +
   \boldsymbol{K}_{ib}
   \approx
   \boldsymbol{0}

$$

2. 固定界面模态满足特征残差：

$$
   \boldsymbol{K}_{ii}\boldsymbol{\Phi}
   -
   \boldsymbol{M}_{ii}\boldsymbol{\Phi}\boldsymbol{\Omega}^2
   \approx
   \boldsymbol{0}

$$

3. 原始 C–B 基的接口块应符合：

$$
   \boldsymbol{T}_{\mathrm{CB},b}
   =
   \begin{bmatrix}
   \boldsymbol{I} & \boldsymbol{0}
   \end{bmatrix}

$$

4. 若最终基声明为质量正交：

$$
   \boldsymbol{\Psi}^T\boldsymbol{M}\boldsymbol{\Psi}
   \approx
   \boldsymbol{I}

$$

5. 若最终刚度声明为对角：

$$
   \boldsymbol{\Psi}^T\boldsymbol{K}\boldsymbol{\Psi}
   \approx
   \operatorname{diag}(\omega_k^2)

$$

### 9.3 刚体属性一致性检查

从节点质量重建：

$$

m_{\mathrm{nodes}}=\sum_jm_j

$$

$$

\boldsymbol{x}_{\mathrm{COM}}
=
\frac{1}{m}
\sum_jm_j\bar{\boldsymbol{u}}_j

$$

$$

\boldsymbol{J}_0
=
\sum_jm_j
\left[
(\bar{\boldsymbol{u}}_j^T\bar{\boldsymbol{u}}_j)\boldsymbol{I}
-
\bar{\boldsymbol{u}}_j\bar{\boldsymbol{u}}_j^T
\right]

$$

验证：

- $m_{\mathrm{nodes}}$ 与 FE 总质量一致；
- 参考系在质心时 $\boldsymbol{x}_{\mathrm{COM}}\approx\boldsymbol{0}$；
- 重建惯量与 FE 报告在同一参考点、同一坐标系下相符。

### 9.4 动态与静态回归测试

建议在导入求解器前后做以下对照：

1. **单模态初位移自由振动**：只给一个模态初值，测得频率应与输入频率一致；
2. **接口单位静位移**：MBD 中施加很慢的接口位移，形变应复现约束模态；
3. **接口单位力静力响应**：降阶结果与全阶 FE 的接口柔度比较；
4. **自由–自由刚体运动**：无外力时整体平动和匀速转动不应产生虚假弹性内力；
5. **重力静挠度**：验证质量分布、方向和力投影；
6. **关节反力测试**：简单悬臂/铰接工况下比较接口反力；
7. **能量测试**：无阻尼无外力时，总能量误差应仅呈现积分器允许的数值耗散；
8. **网格显示测试**：给每个模态小幅正负位移，确认节点编号和分量方向没有置换。

### 9.5 推荐的误差指标

特征残差：

$$

\epsilon_k
=
\frac{
\|\boldsymbol{K}\boldsymbol{\phi}_k
-\omega_k^2\boldsymbol{M}\boldsymbol{\phi}_k\|_2
}{
\|\boldsymbol{K}\boldsymbol{\phi}_k\|_2
+
\omega_k^2\|\boldsymbol{M}\boldsymbol{\phi}_k\|_2
}

$$

模态保证准则（MAC）：

$$

\operatorname{MAC}(\boldsymbol{\phi},\boldsymbol{\psi})
=
\frac{
|\boldsymbol{\phi}^T\boldsymbol{\psi}|^2
}{
(\boldsymbol{\phi}^T\boldsymbol{\phi})
(\boldsymbol{\psi}^T\boldsymbol{\psi})
}

$$

接口柔度相对误差：

$$

\epsilon_{H}
=
\frac{
\|\boldsymbol{H}_{\mathrm{ROM}}-\boldsymbol{H}_{\mathrm{FEM}}\|_F
}{
\|\boldsymbol{H}_{\mathrm{FEM}}\|_F
}

$$

## 10. 必须向求解器开发者确认的问题

建议把下面的问题直接发给开发者，并要求给一个可运行的最小柔性体输入样例：

1. 柔性体导入文件的正式格式、版本和 schema 是什么？
2. 导入器期望原始 C–B 基，还是质量正交化后的最终模态基？
3. 是否显式保留接口自由度，还是所有变形都用统一模态坐标 $\boldsymbol{c}$ 表示？
4. $\boldsymbol{M}_r$、$\boldsymbol{K}_r$ 是否由文件读取，还是由节点质量和振型重新计算？
5. 是否必须提供每节点集中质量？一致质量矩阵如何转换？
6. 模态基的行顺序、节点顺序、索引起点和矩阵存储顺序是什么？
7. 柔性体局部坐标系是否强制位于质心？轴是否必须为主惯性轴？
8. 支持每节点 3 个还是 6 个自由度？壳/梁转动自由度如何导入？
9. 接口 marker 如何定义？是否支持 RBE2/RBE3/MPC 权重和接口转动？
10. 阻尼输入是模态阻尼比、Rayleigh 参数还是完整 $\boldsymbol{C}_r$？
11. 是否需要预计算的惯性不变量？如果需要，字段定义和张量索引顺序是什么？
12. 是否支持应力/应变恢复、接触表面和分布载荷？需要哪些附加矩阵？
13. 四元数的标量分量在前还是在后？局部到全局旋转采用左乘还是右乘？
14. 单位是否写入文件，还是由用户保证整个模型统一？
15. 能否提供一个单柔性悬臂梁 + 转动副的参考算例及期望结果？
16. 最终弹性基是否必须与六个刚体模态质量正交？求解器采用何种浮动参考系约束或 mean-axis 条件？

## 11. 推荐的实际交付流程

### 阶段 A：冻结 FE 与 C–B 定义

1. 冻结 FE 网格、材料、质量附加项和接口集合；
2. 输出内部/接口自由度列表及其顺序；
3. 保存 $\boldsymbol{M}$、$\boldsymbol{K}$ 的版本校验值；
4. 计算固定界面模态和静力约束模态；
5. 生成 $\boldsymbol{T}_{\mathrm{CB}}$、$\boldsymbol{M}_{\mathrm{CB}}$、$\boldsymbol{K}_{\mathrm{CB}}$；
6. 明确是否进行二次正交化，并保留变换矩阵 $\boldsymbol{N}$。

### 阶段 B：转换到 MBD 参考系

1. 确定质量中心和连体坐标系；
2. 转换节点坐标、振型和惯量；
3. 统一单位；
4. 建立节点 ID、接口 ID、marker ID 的映射表；
5. 输出最终 $\boldsymbol{\Psi}$、$\boldsymbol{M}_r$、$\boldsymbol{K}_r$、阻尼。

### 阶段 C：生成求解器文件

1. 严格按导入器 schema 排列数据；
2. 写入版本、单位、坐标系和归一化元数据；
3. 写入接口与 marker 映射；
4. 如有需要，写入节点质量或预积分惯性不变量；
5. 写入可视化/恢复数据；
6. 生成校验和，避免大型矩阵在传输中被截断或转置。

### 阶段 D：分层验证

1. 文件级维度与完整性检查；
2. 矩阵级质量、刚度和特征残差检查；
3. 单柔性体无约束检查；
4. 单接口静力检查；
5. 单关节动力学检查；
6. 最后才进入完整多体系统。

## 12. 建议学习顺序

### 第一阶段：掌握该求解器的运动学

- 欧拉四元数与方向余弦矩阵；
- 浮动坐标系的位置、速度表达；
- 节点模态行块 $\boldsymbol{S}_j$ 的物理意义。

完成标准：能够独立从 $\boldsymbol{r},\boldsymbol{p},\boldsymbol{c}$ 恢复任意节点的全局位置与速度。

### 第二阶段：掌握 C–B

- 内部/接口自由度分块；
- 固定界面模态；
- 静力约束模态；
- C–B 变换；
- 降阶矩阵和二次正交化。

完成标准：能够解释为什么缺少约束模态的模型不是完整 C–B，并能验证约束模态静力残差。

### 第三阶段：掌握 FFRF 动力学

- 刚体–柔性体速度耦合；
- 配置相关质量矩阵；
- 模态内力与广义力；
- 参考系放在质心的作用。

完成标准：能够说明为什么节点质量、坐标和振型可以重建 MBD 所需的惯性耦合。

### 第四阶段：掌握约束和载荷映射

- 柔性节点 marker；
- 约束雅可比中的模态块；
- 节点力到模态力的虚功投影；
- 主从节点和刚性接口映射。

完成标准：能够写出一个柔性节点与地面球铰的约束方程及其对模态坐标的导数。

### 第五阶段：掌握 DAE 求解与验证

- 拉格朗日乘子；
- generalized-$\alpha$ 预测与牛顿修正；
- 残差、切线矩阵和约束缩放；
- 能量、反力和模态响应验证。

完成标准：能够判断错误来自降阶数据、接口映射还是时间积分。

## 13. 一页式交付清单

在把数据发给求解器前，逐项确认：

- [ ] 已明确真实导入格式和版本；
- [ ] 已明确求解器要原始 C–B 基还是正交化基；
- [ ] 固定界面模态和约束模态都存在；
- [ ] 所有 MBD 承载点都包含在接口集合或可恢复集合中；
- [ ] 节点 ID、自由度顺序和索引起点有明确声明；
- [ ] 节点坐标与振型位于同一局部坐标系；
- [ ] 该局部坐标系原点与求解器要求一致；
- [ ] 单位已统一；
- [ ] 总质量、质心和惯量已与 FE 核对；
- [ ] $\boldsymbol{\Psi}$、$\boldsymbol{M}_r$、$\boldsymbol{K}_r$ 采用同一个基；
- [ ] 已确认最终弹性基与刚体模态的正交要求及浮动参考系条件；
- [ ] 阻尼定义与模态归一化一致；
- [ ] 节点质量或等价惯性不变量已提供；
- [ ] 接口 marker 与主从节点权重已提供；
- [ ] 约束模态静力残差和固定界面特征残差合格；
- [ ] 已完成单模态、接口静力、重力、关节和能量回归测试；
- [ ] 已保留生成脚本、输入校验和及转换日志，结果可复现。

## 14. 权威补充资料

- Craig 与 Bampton 的经典方法可从 NASA 技术资料所收录的原始文献和后续综述入手：[A Review of Substructure Coupling Methods for Dynamic Analysis](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770003325.pdf)。
- Altair 对 MBD 柔性体生成的说明明确区分固定界面模态与静力约束模态，并说明接口节点应与 MBD 承载节点一致：[Flexible Body Generation](https://www.help.altair.com/2021/hwsolvers/os/topics/solvers/os/flexible_body_generation_intro_r.htm)。
- 正交化 C–B 模态的工程流程和矩阵关系可参考：[Produce Craig-Bampton Modes for Multi-body Analysis](https://2021.help.altair.com/2021/hwdesktop/hwx/topics/motionview/flexbody_theory_produce_craig_bampton_modes.htm)。
- 一种成熟柔性体数据块的组织方式可参考：[Reference: Flexible Body Data](https://help.altair.com/2022/hwsolvers/ms/topics/solvers/ms/xml-format_89.htm)。它只能作为数据需求的参考，不能视为当前自研求解器的实际格式。

## 15. 最终判断

从开发者理论文档能够确定：该求解器会把你的模态结果作为浮动坐标系柔性体的变形基，用于节点运动恢复、质量耦合、弹性内力、载荷投影、约束雅可比和 DAE 时间积分。

从开发者理论文档不能确定：它究竟以什么文件格式、字段和矩阵变体读取这些数据。

因此目前最稳妥的技术方案是：准备一份包含“参考系 + 节点坐标 + 节点质量或惯性不变量 + 接口映射 + 最终位移基 + 降阶质量/刚度/阻尼 + 频率与归一化元数据”的完整中间数据包，再根据导入器 schema 做一次无损转换。不要先压缩成只剩频率和对角矩阵的简化结果。
