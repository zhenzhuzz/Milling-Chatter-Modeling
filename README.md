
# milling-chatter-modeling

## 项目简介
本项目面向 **铣削加工中的自激振动（颤振，Chatter）动力学建模、稳定性分析与仿真**，基于再生效应的 **时滞微分方程 (Delay Differential Equation, DDE)**，结合 MATLAB 程序实现了从基础振动现象到铣削颤振数值模拟与控制策略验证的全过程。

核心功能：
1. 理解不同振动类型（自由/受迫/自激）的动力学特征。
2. 构建并求解铣削过程的再生型颤振动力学模型。
3. 实现变转速铣削的时域仿真，并可视化颤振发展与抑制过程。
4. 为颤振监测与调速抑制策略提供仿真验证平台。

---

## 理论背景

### 铣削颤振机理
铣削颤振是一种 **自激振动**，由加工系统内部的能量反馈回路驱动，无需外部周期激励。其核心机制是 **再生效应（Regenerative Effect）**：

$$
h_j(t) = h_{j,\text{stat}}(t) + h_{j,\text{dyn}}(t)
$$

其中：
$$
h_{j,\text{stat}}(t) = f_z \sin \phi_j(t)
$$

$$
h_{j,\text{dyn}}(t) =
\begin{bmatrix}
\sin\phi_j(t) & \cos\phi_j(t)
\end{bmatrix}
\left[ \mathbf{v}(t) - \mathbf{v}(t-\tau) \right]
$$

- **稳定能量源**：主轴电机稳定供能。
- **调制机制 M**：前一刀齿在工件表面留下的波纹，改变后一刀齿的瞬时切削厚度。
- **周期性力**：切削厚度波动使切削力周期变化，反作用于刀具–工件系统。
- **反馈回路**：振动 → 切削厚度变化 → 切削力变化 → 更大振动。

---

### 铣削过程动力学建模
将主轴–刀柄–刀具系统简化为二维质量–弹簧–阻尼模型：

$$
\dot{\mathbf{x}}(t) = \mathbf{A}\mathbf{x}(t) + \mathbf{B} \mathbf{F}(t)
$$

$$
\mathbf{v}(t) = \mathbf{C} \mathbf{x}(t)
$$

切削力模型（经验指数形式）：

$$
F_{\text{tang},j} = g_j(\phi_j) K_t a_p h_j(t)^{x_F}
$$

$$
F_{\text{rad},j} = g_j(\phi_j) K_r a_p h_j(t)^{x_F}
$$

综合得到含时滞项的铣削动力学方程：

$$
\dot{\mathbf{x}}(t) =
\mathbf{A} \mathbf{x}(t) +
\mathbf{B} a_p \sum_{j=0}^{z-1} g_j(\phi_j(t))
\left\{
h_{j,\text{stat}}(t) + \dots
\right\}
$$

这是一个 **变时滞、非线性、周期系数的时滞微分方程**（TV-DDE）。

---

### 稳定性分析
- **Floquet 理论**：分析周期解的稳定性，计算 Floquet 乘子 $\mu$。
- **判据**：
  - $|\mu|<1$：稳定
  - $|\mu|>1$：失稳 → 颤振
- **半离散法**：将一个齿周期离散为 $N$ 小段，构造有限维状态转移矩阵近似 Floquet 算子，生成 **稳定性叶瓣图 (SLD)**。

示例稳定性叶瓣图（不同转速–切削深度下的稳定/不稳定区域）：
![Stability Lobes Example](Media/stability_lobes_example.png)

---

### 颤振类型与分岔
- **次级 Hopf 分岔**：出现新频率成分，准周期运动（颤振常见形式）。
- **倍周期分岔**：出现 $1/2$ 或 $1/4$ 主轴转速频率成分。
- 铣削过程主要关注这两类分岔。

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

* MATLAB R2020a+
    
* Signal Processing Toolbox
    

* * *

参考文献
----

1. Altintas, Y. Manufacturing Automation, CUP, 2012.
    
2. Tlusty, J. Dynamics of High-Speed Milling, ASME J. Eng. Ind., 1985.
    
3. Tobias, S.A. Machine-Tool Vibration, Blackie & Son, 1965.
    

```

---

这样这个 README：  
- 所有公式都用 `$$...$$` 包裹，可以在支持 MathJax 的环境渲染。  
- 论文第三、四章的关键推导、公式、图片、直观类比都已经放进去。  
- 代码运行方法和功能介绍都一并包含。  
- GitHub 用户即使不看论文，也能直接理解理论背景和代码作用。  

如果你愿意，我还可以帮你**加一个 GitHub Pages 版本**，让这些公式和图片在网页端渲染得非常漂亮，同时还能做成一个在线的项目文档。这样别人看起来就和读在线教材一样。  

你要我帮你做这个 Pages 版吗？这样 README 就是简略版，网页是完整版。
```
