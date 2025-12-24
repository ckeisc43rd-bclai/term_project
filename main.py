import numpy as np
import matplotlib.pyplot as plt
# 明確匯入 Axes3D 以防止在某些環境下出現 'Unknown projection' 錯誤
from mpl_toolkits.mplot3d import Axes3D
from abc import ABC, abstractmethod

# ==========================================
# 1. 物理常數與環境設定
# ==========================================
G = 9.81          # 重力加速度 (m/s^2)
RHO = 1.225       # 空氣密度 (kg/m^3), 海平面標準
DT = 0.01         # 模擬時間步長 (s)

# ==========================================
# 2. 拋體類別定義 (物理引擎)
# ==========================================

class Projectile(ABC):
    def __init__(self, mass, drag_coeff, position, velocity):
        self.mass = mass
        self.cd = drag_coeff
        # 狀態向量 state = [x, y, z, vx, vy, vz]
        self.state = np.array([*position, *velocity], dtype=float)
        self.trajectory = [self.state.copy()] # 記錄軌跡
        self.time = [0.0]

    @abstractmethod
    def get_projected_area(self, velocity_vector):
        """由子類別實作：根據速度向量計算投影截面積"""
        pass

    def get_derivative(self, state):
        """計算導數 (用於 RK4 積分)"""
        x, y, z, vx, vy, vz = state
        v_vec = np.array([vx, vy, vz])
        v_mag = np.linalg.norm(v_vec)

        # 重力加速度
        ax, ay, az = 0.0, 0.0, -G
        area = 0.0

        if v_mag > 0:
            # 計算迎風面積與空氣阻力
            area = self.get_projected_area(v_vec)
            drag_force_mag = 0.5 * RHO * self.cd * area * (v_mag**2)

            # 阻力方向與速度方向相反
            fx_d = -drag_force_mag * (vx / v_mag)
            fy_d = -drag_force_mag * (vy / v_mag)
            fz_d = -drag_force_mag * (vz / v_mag)

            # 加入阻力產生的加速度 (F=ma => a=F/m)
            ax += fx_d / self.mass
            ay += fy_d / self.mass
            az += fz_d / self.mass

        return np.array([vx, vy, vz, ax, ay, az]), area

    def rk4_step(self, dt):
        """執行一步 Runge-Kutta 4 數值積分"""
        state = self.state
        d1, area = self.get_derivative(state)
        d2, _ = self.get_derivative(state + 0.5 * dt * d1)
        d3, _ = self.get_derivative(state + 0.5 * dt * d2)
        d4, _ = self.get_derivative(state + dt * d3)

        self.state = state + (dt / 6.0) * (d1 + 2*d2 + 2*d3 + d4)
        self.trajectory.append(self.state.copy())
        self.time.append(self.time[-1] + dt)

    def calculate_drag_history(self):
        """重新計算整個軌跡的歷史阻力值 (用於繪圖)"""
        drags = []
        traj = np.array(self.trajectory)
        for state in traj:
            v_vec = state[3:6]
            v_mag = np.linalg.norm(v_vec)
            if v_mag == 0:
                drags.append(0.0)
            else:
                area = self.get_projected_area(v_vec)
                f_drag = 0.5 * RHO * self.cd * area * (v_mag**2)
                drags.append(f_drag)
        return np.array(drags)

class SphereProjectile(Projectile):
    """球體：截面積固定"""
    def __init__(self, radius, mass, drag_coeff, pos, vel):
        super().__init__(mass, drag_coeff, pos, vel)
        self.radius = radius
        self.cross_section = np.pi * (radius ** 2)

    def get_projected_area(self, velocity_vector):
        return self.cross_section

class CubeProjectile(Projectile):
    """正方體：姿態固定，截面積隨速度方向改變"""
    def __init__(self, side_length, mass, drag_coeff, pos, vel):
        super().__init__(mass, drag_coeff, pos, vel)
        self.L = side_length

    def get_projected_area(self, velocity_vector):
        vx, vy, vz = velocity_vector
        v_mag = np.linalg.norm(velocity_vector)
        if v_mag == 0: return self.L ** 2

        # 立方體公式簡化：L^2 * (|nx| + |ny| + |nz|)
        nx_comp = abs(vx) / v_mag
        ny_comp = abs(vy) / v_mag
        nz_comp = abs(vz) / v_mag
        area = (self.L ** 2) * (nx_comp + ny_comp + nz_comp)
        return area

class RectangularPrismProjectile(Projectile):
    """長方體：姿態固定"""
    def __init__(self, lx, ly, lz, mass, drag_coeff, pos, vel):
        super().__init__(mass, drag_coeff, pos, vel)
        self.Lx = lx
        self.Ly = ly
        self.Lz = lz

    def get_projected_area(self, velocity_vector):
        vx, vy, vz = velocity_vector
        v_mag = np.linalg.norm(velocity_vector)
        if v_mag == 0: return self.Lx * self.Ly

        area_face_x = self.Ly * self.Lz
        area_face_y = self.Lx * self.Lz
        area_face_z = self.Lx * self.Ly

        nx_comp = abs(vx) / v_mag
        ny_comp = abs(vy) / v_mag
        nz_comp = abs(vz) / v_mag

        total_area = (area_face_x * nx_comp) + (area_face_y * ny_comp) + (area_face_z * nz_comp)
        return total_area

class TetrahedronProjectile(Projectile):
    """正四面體：4 個面"""
    def __init__(self, side_length, mass, drag_coeff, pos, vel):
        super().__init__(mass, drag_coeff, pos, vel)
        self.a = side_length
        # 單面面積 S = (sqrt(3)/4) * a^2
        self.face_area = (np.sqrt(3) / 4) * (self.a ** 2)

        # 定義四個面的法向量 (基於內接於立方體的幾何結構)
        # 歸一化常數 1/sqrt(3)
        inv_sqrt3 = 1.0 / np.sqrt(3)
        self.normals = np.array([
            [1, 1, 1],
            [1, -1, -1],
            [-1, 1, -1],
            [-1, -1, 1]
        ]) * inv_sqrt3

    def get_projected_area(self, velocity_vector):
        v_mag = np.linalg.norm(velocity_vector)
        if v_mag == 0: return self.face_area # 靜止時隨意回傳一個值

        # 單位速度向量
        v_hat = velocity_vector / v_mag

        # 計算投影面積總和: 0.5 * sum(Area_i * |v_hat dot n_i|)
        total_proj = 0.0
        for n in self.normals:
            total_proj += abs(np.dot(v_hat, n))

        return 0.5 * self.face_area * total_proj

class OctahedronProjectile(Projectile):
    """正八面體：8 個面"""
    def __init__(self, side_length, mass, drag_coeff, pos, vel):
        super().__init__(mass, drag_coeff, pos, vel)
        self.a = side_length
        # 單面面積 S = (sqrt(3)/4) * a^2
        self.face_area = (np.sqrt(3) / 4) * (self.a ** 2)

        # 定義八個面的法向量 (對應八個卦限)
        # 向量為 (+-1, +-1, +-1) / sqrt(3)
        inv_sqrt3 = 1.0 / np.sqrt(3)
        self.normals = []
        for x in [1, -1]:
            for y in [1, -1]:
                for z in [1, -1]:
                    self.normals.append([x, y, z])
        self.normals = np.array(self.normals) * inv_sqrt3

    def get_projected_area(self, velocity_vector):
        v_mag = np.linalg.norm(velocity_vector)
        if v_mag == 0: return self.face_area

        v_hat = velocity_vector / v_mag

        # 計算投影面積總和
        total_proj = 0.0
        for n in self.normals:
            total_proj += abs(np.dot(v_hat, n))

        return 0.5 * self.face_area * total_proj

# ==========================================
# 3. 模擬與計算邏輯
# ==========================================

def run_simulation(projectile):
    """執行單次模擬直到落地 (z < 0)"""
    max_steps = 15000
    step = 0
    while projectile.state[2] >= 0 and step < max_steps:
        projectile.rk4_step(DT)
        step += 1
    return np.array(projectile.trajectory)

def simulate_single_case(shape_class, shape_args, mass, cd, v0, azi_deg, elev_deg):
    """模擬單一案例並回傳拋體物件"""
    elev = np.radians(elev_deg)
    azi = np.radians(azi_deg)
    vx = v0 * np.cos(elev) * np.cos(azi)
    vy = v0 * np.cos(elev) * np.sin(azi)
    vz = v0 * np.sin(elev)
    start_vel = [vx, vy, vz]

    proj = shape_class(*shape_args, mass, cd, [0,0,0], start_vel)
    run_simulation(proj)
    return proj

def find_optimal_case(shape_class, shape_args, mass, cd, v0, azi_deg, name):
    """掃描仰角以尋找最大射程"""
    print(f"  -> 正在優化分析 {name} ...")
    test_angles = np.arange(10, 81, 2)
    #間隔兩度
    max_range = 0
    best_proj = None
    best_angle = 0

    angles_list = []
    ranges_list = []

    for deg in test_angles:
        proj = simulate_single_case(shape_class, shape_args, mass, cd, v0, azi_deg, deg)
        landing_pos = proj.trajectory[-1]
        dist = np.linalg.norm(landing_pos[:2])

        angles_list.append(deg)
        ranges_list.append(dist)

        if dist > max_range:
            max_range = dist
            best_proj = proj
            best_angle = deg

    return {
        'name': name,
        'best_proj': best_proj,
        'best_angle': best_angle,
        'max_range': max_range,
        'angles': angles_list,
        'ranges': ranges_list
    }

# ==========================================
# 4. 繪圖函式庫 (Matplotlib)
# ==========================================

def plot_trajectory_3d(ax, proj, title, color='blue', label=None):
    """繪製 3D 軌跡圖"""
    traj = np.array(proj.trajectory)
    if len(traj) == 0:
        return
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color=color, label=label, linewidth=2)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    if label:
        ax.legend(loc='best', fontsize='small')

def plot_drag_vs_x(ax, proj, title, color='red'):
    """繪製 阻力 vs X軸位移 圖"""
    traj = np.array(proj.trajectory)
    drags = proj.calculate_drag_history()
    xs = traj[:, 0]

    # 對齊長度
    min_len = min(len(xs), len(drags))
    ax.plot(xs[:min_len], drags[:min_len], color=color)
    ax.set_xlabel('Distance X (m)')
    ax.set_ylabel('Drag Force (N)')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)

# ==========================================
# 5. 主程式執行區塊
# ==========================================

if __name__ == "__main__":
    # --- 參數設定 ---
    MASS = 2.0
    CD = 0.5
    V0 = 80.0
    AZI = 0.0

    # 定義形狀清單 (加入正四面體與正八面體)
    # 注意：邊長設定是隨意的，為了便於比較，大約設定在 0.2~0.3 m 左右
    SHAPES = [
        (SphereProjectile, (0.15,), "Sphere"),
        (CubeProjectile, (0.2,), "Cube"),
        (RectangularPrismProjectile, (0.5, 0.3, 0.05), "Rectangular Prism"),
        (TetrahedronProjectile, (0.3,), "Tetrahedron"),
        (OctahedronProjectile, (0.25,), "Octahedron")
    ]

    print("="*40)
    print("拋體運動最佳化分析程式已啟動 (含多面體)")
    print("="*40)

    # === 執行模擬 ===
    results = []
    for cls, args, name in SHAPES:
        res = find_optimal_case(cls, args, MASS, CD, V0, AZI, name)
        results.append(res)
        print(f"    {name} 最佳仰角: {res['best_angle']} 度, 射程: {res['max_range']:.2f} m")

    # === 建立 6x2 圖表網格 (總共 5 個物體 + 1 個總覽) ===
    fig = plt.figure(figsize=(16, 30))
    # 調整子圖間距: hspace(高), wspace(寬)
    plt.subplots_adjust(hspace=0.4, wspace=0.2, top=0.96, bottom=0.04)

    # --- 第一列: 總覽 ---
    colors = ['blue', 'orange', 'green', 'purple', 'brown']

    # 1.1 合併 3D 軌跡
    ax1 = fig.add_subplot(6, 2, 1, projection='3d')
    for i, res in enumerate(results):
        plot_trajectory_3d(ax1, res['best_proj'], "Comparison of Best Trajectories",
                           color=colors[i], label=f"{res['name']} ({res['best_angle']}°)")

    # 1.2 角度 vs 射程 (2D)
    ax2 = fig.add_subplot(6, 2, 2)
    for i, res in enumerate(results):
        ax2.plot(res['angles'], res['ranges'], color=colors[i], label=res['name'], marker='o', markersize=3)
        ax2.plot(res['best_angle'], res['max_range'], '*', color=colors[i], markersize=12)
    ax2.set_xlabel("Launch Angle (deg)")
    ax2.set_ylabel("Range (m)")
    ax2.set_title("Range vs Launch Angle")
    ax2.grid(True)
    ax2.legend()

    # --- 第2~6列: 個別物體詳細資訊 (左圖: 軌跡, 右圖: 阻力) ---
    for i, res in enumerate(results):
        row = i + 1 # 從第2列開始 (index 1)

        # 左圖: 3D 軌跡
        ax_traj = fig.add_subplot(6, 2, row*2 + 1, projection='3d')
        plot_trajectory_3d(ax_traj, res['best_proj'], f"{res['name']} Best Trajectory", color=colors[i])

        # 右圖: 阻力 vs 位移
        ax_drag = fig.add_subplot(6, 2, row*2 + 2)
        plot_drag_vs_x(ax_drag, res['best_proj'], f"{res['name']} Drag vs Distance", color=colors[i])

    print("\n分析完成，正在顯示圖表...")
    plt.show()