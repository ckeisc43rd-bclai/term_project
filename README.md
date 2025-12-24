
# 🚀 Multi-Shape Projectile Motion Optimizer  
**多形狀拋體運動與最佳化模擬器（考慮空氣阻力與動態截面積）**

本專案是一個 **基於 Python 的物理數值模擬工具**，用來研究**不同幾何形狀的物體**在考慮空氣阻力時的拋體運動行為，並進一步尋找 **最大射程與最佳發射角**。

與傳統拋體模型不同，本模擬 **不假設截面積為常數**。即使物體在飛行過程中**姿態固定、不旋轉**，由於速度向量方向會隨時間改變，迎風面的**投影截面積（Dynamic Cross-Sectional Area）**仍會發生變化，進而顯著影響阻力與飛行距離。

---

## ✨ 專案特色

- **多種幾何形狀支援**
  - 球體 (Sphere)
  - 正方體 (Cube)
  - 長方體 (Rectangular Prism)
  - 正四面體 (Tetrahedron)
  - 正八面體 (Octahedron)

- **動態空氣阻力模型**
  - 即時計算物體相對於速度方向的投影截面積
  - 模擬非球形物體的真實氣動效應

- **高精度數值積分**
  - 採用 Runge–Kutta 4th Order (RK4)
  - 相較歐拉法大幅降低長時間模擬誤差

- **自動最佳化搜尋**
  - 掃描發射仰角（10°–80°）
  - 對每一種形狀找出最大射程與最佳發射角

- **完整視覺化分析**
  - 3D 飛行軌跡
  - 射程 vs. 發射仰角
  - 空氣阻力 vs. 飛行距離

---

## 📚 物理與數學模型

### 空氣阻力模型

Fd = -1/2 * rho * Cd * A_proj * |v| * v

rho = 1.225 kg/m^3

### 動態投影截面積

A_proj = 1/2 * sum(S_i * |v_hat · n_i|)

### 數值積分

使用 RK4 方法更新狀態向量。

---

## ⚙️ 安裝與使用

```bash
pip install numpy matplotlib
```

```bash
python main.py
```

---
## 程式架構

定義所有物體物體需要之運算規則後分別定義五個物體的細項

---
## 開發過程

給gemini構想執行後修正錯誤

---
## 參考資料
[llm連結](https://gemini.google.com/share/87454cacece2)
---
## 📌 作者
Pao Cheng Lai
Chemical Engineering / Physics Numerical Simulation Project
