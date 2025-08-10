
# milling-chatter-modeling

## 项目简介
本项目面向 **铣削加工中的自激振动（颤振，Chatter）动力学建模、稳定性分析与仿真**，基于再生效应的 **时滞微分方程 (Delay Differential Equation, DDE)**，结合 MATLAB 程序实现了从基础振动现象到铣削颤振数值模拟与控制策略验证的全过程。

核心功能：
1. 理解不同振动类型（自由/受迫/自激）的动力学特征。
2. 构建并求解铣削过程的再生型颤振动力学模型。
3. 实现变转速铣削的时域仿真，并可视化颤振发展与抑制过程。
4. 为颤振监测与调速抑制策略提供仿真验证平台。

---

## 理论背景：铣削颤振动力学建模与稳定性分析

### 1. 再生效应机理
铣削是具有时滞特性的动态过程：相邻刀齿在同一位置切削时，由于刀具–工件的弹性变形，会产生**动态位移差**，改变切屑厚度，引发自激振动（再生颤振）。

总切屑厚度：

$$
h_j(t) = h_{j,\text{stat}}(t) + h_{j,\text{dyn}}(t)
$$

静态切屑厚度：

$$
h_{j,\text{stat}}(t) = f_z \sin \phi_j(t)
$$

动态切屑厚度：

$$
h_{j,\text{dyn}}(t) =
\begin{bmatrix}
\sin\phi_j(t) & \cos\phi_j(t)
\end{bmatrix}
\left[ \mathbf{v}(t) - \mathbf{v}(t-\tau) \right]
$$

其中 $\tau=\frac{60}{zn}$ 为相邻刀齿通过周期，$z$ 为齿数，$n$ 为主轴转速。

---

### 2. 切削力模型
单齿切向/径向切削力（经验指数型）：

$$
F_{\text{tang},j} = g_j(\phi_j) K_t a_p h_j(t)^{x_F}
$$

$$
F_{\text{rad},j} = g_j(\phi_j) K_r a_p h_j(t)^{x_F}
$$

$x_F \in (0,1]$ 表示非线性程度，$g_j(\phi_j)$控制刀齿在切削区间的作用。

总切削力向量：

$$
{\mathbf{F}}_{\mathrm{t}}\left( t\right)  = {a}_{\mathrm{p}}\mathop{\sum }\limits_{{j = 0}}^{{z - 1}}{g}_{j}\left( {{\phi }_{j}\left( t\right) }\right) {\left( {h}_{j,\text{ stat }}\left( t\right)  + {h}_{j,\text{ dyn }}\left( t\right) \right) }^{{x}_{F}}\mathbf{S}\left( t\right) \left\lbrack  \begin{array}{l} {K}_{\mathrm{t}} \\  {K}_{\mathrm{r}} \end{array}\right\rbrack
$$

$$
\mathbf{S}\left( t\right)  = \left\lbrack  \begin{matrix}  - \cos {\phi }_{j}\left( t\right) &  - \sin {\phi }_{j}\left( t\right) \\  \sin {\phi }_{j}\left( t\right) &  - \cos {\phi }_{j}\left( t\right)  \end{matrix}\right\rbrack
$$

为旋转坐标系到机床坐标系的几何变换矩阵。

---

### 3. 主轴–刀具系统动力学
二维质量–弹簧–阻尼模型（$x$、$y$方向）：

$$
\dot{\mathbf{x}}(t) = \mathbf{A} \mathbf{x}(t) + \mathbf{B} \mathbf{F}(t)
$$

$$
\mathbf{v}(t) = \mathbf{C} \mathbf{x}(t)
$$

与切削力模型耦合，得到含时滞的铣削动力学方程（TV-DDE）：

$$
\dot{\mathbf{x}}\left( t\right)  = \mathbf{A}\mathbf{x}\left( t\right)  + \mathbf{B}{a}_{\mathrm{p}}\mathop{\sum }\limits_{{j = 0}}^{{z - 1}}{g}_{j}\left( {{\phi }_{j}\left( t\right) }\right) {\left\{  {h}_{j,\text{ stat }}\left( t\right)  + \left\lbrack  \begin{array}{ll} \sin {\phi }_{j}\left( t\right) & \cos {\phi }_{j}\left( t\right)  \end{array}\right\rbrack  \left( \mathbf{C}\mathbf{x}\left( t\right)  - \mathbf{C}\mathbf{x}\left( t - {\tau }_{j}\left( t\right) \right) \right) \right\}  }^{{x}_{F}}\mathbf{S}\left( t\right) \left\lbrack  \begin{matrix} {K}_{t} \\  {K}_{r} \end{matrix}\right\rbrack
$$


---

### 4. 稳定性分析流程
1. **周期解与线性化**：静态切屑厚度周期性 → 刀具运动周期解 $v_p(t)$；小扰动 $v_u(t)$ 线性化为：

$$
\dot{\widetilde{\mathbf{x}}}(t) =
\mathbf{P}(t) \widetilde{\mathbf{x}}(t) +
\mathbf{Q}(t) \widetilde{\mathbf{x}}(t-\tau)
$$

2. **Floquet 理论**：构造单周期转移算子 $\mathbf{\Phi}_T$，其特征值 $\mu$ 为 Floquet 乘子：
   - $|\mu|<1$ 稳定
   - $|\mu|>1$ 失稳（颤振）

3. **半离散法**：将周期离散为 $N$ 小段，近似 $\mathbf{P}(t),\mathbf{Q}(t)$ 为分段常数，构造有限维转移矩阵 $\mathbf{U}$ 近似 $\mathbf{\Phi}_T$，求解临界条件 $|\mu|=1$ → **稳定性叶瓣图**（SLD）。

---

### 5. 分岔类型与频率特征
- **次级 Hopf 分岔**：$\mu$ 成对复数跨单位圆 → 准周期运动，出现基本颤振频率 $f_c$。
- **倍周期分岔**：$\mu=-1$ → 周期加倍，出现 $f_{\mathrm{sp}}/2$ 等分数谐波。

颤振频率与 Floquet 乘子关系：

$$
f_c = \frac{Im(\ln \mu)}{2\pi T}
$$

频谱成分：
- 主轴转速谐波 $kf_{\mathrm{sp}}$
- 刀齿通过频率谐波 $kf_{\mathrm{tpe}}$
- 系统固有频率 $f_d$
- 分岔特征频率 $f_H, f_{PD}$

---

### 6. 颤振演变阶段
1. **稳定切削**：$f_{\mathrm{sp}}$、$f_{\mathrm{tpe}}$ 及谐波占主导，振幅低。
2. **颤振萌生**：$f_H$ 或 $f_{PD}$ 幅值上升，接近 $f_d$；可见分数谐波。
3. **颤振完全发展**：主导颤振频率能量大幅增强，振幅剧增，表面出现波纹。

---

以上建模与分析为项目中 **MATLAB 时域仿真**、**稳定性叶瓣预测**、**颤振监测与调速策略** 提供了理论基础。


---

## 直观类比
- **Stick–Slip 摩擦振动**：手推橡皮块–弹簧系统
- **小提琴**：琴弓稳定拉力 + 弦–琴体反馈
- **长笛**：稳定气流 + 声学反馈

这些系统都依赖于：
1. 稳定能量输入
2. 调制机制
3. 周期性反馈力
4. 能量积累到达阈值 → 自激振荡

---

## 代码说明

### 1. 铣削颤振仿真
- **文件**：`m050_milling_Time_domain_Sim_01.m`
- 主函数：

  milling_simulation(rpm, cutDepth, feedPerTooth, numTeeth, simTime)

* 功能：
    
    * 构建 chatterDDE
        
    * 支持多段转速切换（变转速铣削）
        
    * 输出位移曲线、FFT 频谱、Poincaré 截面
        
* 示例：
    

    milling_simulation([4500, 8200, 10000], 2e-3, 0.05e-3, 4, 1.0);

    

* * *

### 2. 振动类型动画

这三个 MATLAB 脚本展示了 stick–slip 模型下的三种典型振动现象，通过橡皮块–弹簧–摩擦模型可视化动力学过程。

* **`m034_Free_eraser_stick_slip_animation_Subplot_disp_force.m`**
    
    * 自由振动：无外力，初始位移/速度 → 指数衰减振荡。
        
* **`m033_Forced_eraser_stick_slip_animation_Subplot_disp_force.m`**
    
    * 受迫振动：外部周期力驱动，稳态振幅取决于激励频率，共振时最大。
        
* **`m032_eraser_stick_slip_animation_Subplot_disp_force_variation.m`**
    
    * 自激振动：无外部周期力，内部反馈+稳定能量源 → 振幅自增至饱和。
        

运行：

```matlab
m034_Free_eraser_stick_slip_animation_Subplot_disp_force;
```

将生成位移–速度–力随时间变化的动画。

* * *

颤振监测与调速策略（第四章摘要）
----------------

实时采集主轴加速度信号 → FFT → 陷波滤波去除主轴与刀齿通过频率 → 检测新增频率成分 → 若接近固有频率/分岔频率，则判定颤振 → 调整主轴转速至稳定叶瓣中心。

转速计算公式：

$$k_{\text{new}} = \left\lfloor \frac{f_{\text{chat}} \cdot 60}{z n} \right\rfloor$$ $$n_{\text{new}} = \frac{60 f_{\text{chat}}}{k_{\text{new}} z}$$

* * *

运行环境
----

* MATLAB R2023a及以上
    
* Signal Processing Toolbox
    

* * *

参考文献
----

1. Altintas, Y. Manufacturing Automation, CUP, 2012.
    
2. Tlusty, J. Dynamics of High-Speed Milling, ASME J. Eng. Ind., 1985.
    
3. Tobias, S.A. Machine-Tool Vibration, Blackie & Son, 1965.
    

