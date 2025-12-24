多形狀拋體運動與最佳化模擬器 (Multi-Shape Projectile Motion Optimizer)這是一個基於 Python 的物理模擬專案，旨在探討不同幾何形狀的物體（球體、正方體、長方體、正四面體、正八面體）在考慮空氣阻力時的拋體運動軌跡。本程式的核心特色在於計算動態截面積 (Dynamic Cross-Sectional Area)：即使物體在飛行過程中保持姿態固定（不旋轉），由於速度向量方向的改變，其迎風面的投影面積仍會隨時間變化，進而影響空氣阻力的大小與飛行距離。🚀 專案功能多種幾何形狀支援：內建球體、正方體、長方體、正四面體、正八面體的幾何模型。高精度數值積分：採用 Runge-Kutta 4th Order (RK4) 方法求解微分方程，比傳統歐拉法更精確。動態阻力計算：即時計算物體相對於氣流速度向量的投影面積，模擬真實的非球形物體空氣動力特性。自動最佳化搜尋：自動掃描 10° 至 80° 的發射仰角，找出每種形狀的「最大射程」與「最佳發射角」。視覺化數據分析：生成 3D 飛行軌跡圖。繪製「射程 vs. 仰角」關係圖。繪製「空氣阻力 vs. 飛行距離」變化圖，觀察形狀對阻力的影響。📚 技術原理1. 空氣阻力模型物體在飛行中受到的空氣阻力 $F_d$ 依循二次阻力定律：$$\vec{F}_d = -\frac{1}{2} \rho C_d A_{proj} |\vec{v}| \vec{v}$$$\rho$: 空氣密度 ($1.225 \, kg/m^3$)$C_d$: 阻力係數 (本模擬假設為常數)$v$: 物體速度$A_{proj}$: 投影截面積 (Projected Area)，這是本專案的計算核心。2. 動態投影面積演算法對於非球形物體（如立方體），當其姿態固定但速度方向改變時，氣流「看到」的面積會改變。對於任意凸多面體，在速度單位向量 $\hat{v}$ 方向上的投影面積計算公式為：$$A_{proj} = \frac{1}{2} \sum_{i} S_i |\hat{v} \cdot \vec{n}_i|$$其中：$S_i$ 為第 $i$ 個面的面積。$\vec{n}_i$ 為第 $i$ 個面的法向量。內積 $\hat{v} \cdot \vec{n}_i$ 代表速度向量在該面法向量上的投影分量。3. 數值積分 (RK4)為了解決非線性的運動方程式，我們將時間切割為微小的 $\Delta t$ (0.01s)，並使用 RK4 方法更新狀態：$$\vec{y}_{n+1} = \vec{y}_n + \frac{\Delta t}{6}(k_1 + 2k_2 + 2k_3 + k_4)$$這能有效減少長時間模擬下的累積誤差。🛠️ 程式架構說明程式碼主要由三個部分組成：1. 物理引擎核心 (Projectile 類別)Projectile (ABC): 抽象基底類別，定義了質量、位置、速度等屬性，以及 rk4_step (積分步進) 和 get_derivative (計算受力與加速度) 方法。get_projected_area: 抽象方法，強制所有子類別必須實作自己的面積計算邏輯。2. 形狀實作 (幾何子類別)SphereProjectile: 截面積恆為 $\pi r^2$。CubeProjectile: 根據 6 個面法向量計算投影。RectangularPrismProjectile: 長方體，需考慮三個軸向不同的面面積。TetrahedronProjectile: 正四面體，定義 4 個特定角度的面法向量。OctahedronProjectile: 正八面體，定義 8 個卦限方向的面法向量。3. 模擬控制與繪圖 (Main 區塊)run_simulation: 執行單次飛行直到落地。find_optimal_case: 迴圈測試不同仰角，尋找最大射程。plot_* 函式: 使用 Matplotlib 繪製 3D 軌跡與 2D 數據圖表。⚙️ 如何使用與參數設定1. 安裝需求套件請確保您的 Python 環境已安裝以下套件：pip install numpy matplotlib
2. 執行模擬直接執行 Python 腳本即可：python projectile_sim.py
3. 修改模擬參數您可以在程式碼最下方的 if __name__ == "__main__": 區塊中修改以下參數：# --- 物理參數 ---
MASS = 2.0        # 物體質量 (kg)
CD = 0.5          # 阻力係數 (Drag Coefficient)
V0 = 80.0         # 初始發射速度 (m/s)
AZI = 0.0         # 發射方位角 (0度代表沿 X 軸發射)

# --- 形狀設定 ---
# 格式: (類別名, (尺寸參數...), "顯示名稱")
SHAPES = [
    (SphereProjectile, (0.15,), "Sphere"),        # 半徑 0.15m
    (CubeProjectile, (0.2,), "Cube"),             # 邊長 0.2m
    (RectangularPrismProjectile, (0.5, 0.3, 0.05), "Rectangular Prism"), # 長寬高
    (TetrahedronProjectile, (0.3,), "Tetrahedron"), # 邊長
    (OctahedronProjectile, (0.25,), "Octahedron")   # 邊長
]
