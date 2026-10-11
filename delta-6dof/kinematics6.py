"""
Delta 6DOF 运动学原型（P0）
============================
构型：三主臂 + 三副臂（六轴 Delta，方案①，见 DESIGN.md）
  主链 i=1..3：主动臂 L + 从动杆 l，平台边缘铰点半径 r，基座半径 R
  副链 j=1..3：小型二连杆（副主动臂 La + 副连杆 lb），平台中心铰点半径 ra，基座半径 Rac

坐标系：基座系原点在顶板中心，Z 轴【向下】为正（沿用 3DOF 约定）。
姿态：ZYX 固定角 RPY，R = Rz(C)·Ry(B)·Rx(A)。

单位约定（全模块统一）：
  位姿 pose6 = (X, Y, Z, A, B, C)，位置 mm，A/B/C 【弧度】（接口层转换见 pose_deg/pose_rad）
  电机角 theta = 6 元组【弧度】，顺序 = 主1..主3 + 副1..副3

链方程（每链独立、闭式）：
  链 k 的基座铰位于方位角 φk（0/120/240°），切向单位轴 t，径向 n。
  肘点 Ek(θ) = (Rs + La·cosθ)·n + La·sinθ·ẑ   （Rs=基座分布半径，La=主动臂长）
  平台铰点 Ak = p + Rot·ak
  约束 |Ak − Ek|² = Lb²，在链平面内分解 u = Ak·n − Rs, v = Ak·t, w = Ak·z：
      u·cosθ + w·sinθ = (La² + u² + v² + w² − Lb²) / (2La) = c
      m·cos(θ − α) = c，m = hypot(u,w)，α = atan2(w,u) → θ = α ± acos(c/m)
  有效性：|c| ≤ m 且 sinθ > 0（肘点在顶板下方）；多解取距参考角最近者。

FK：6 平方距离残差 + 解析雅可比（RPY 偏导用叉积形式）+ 牛顿迭代（温启动）。
前馈：θ̇ = (Ak−Ek)·(ṗ + d(Rot·ak)/dt) / [(Ak−Ek)·dEk/dθ]，d(Rot·a)/dt = dA·Ȧ + dB·Ḃ + dC·Ċ。
"""
import math

# ---------------------------------------------------------------- 机械参数
# 注：6DOF 主臂较 3DOF 加长（L+l = 400 vs 350）——姿态偏移（铰点随 Rot·a 移动）
# 挤占杆长预算，沿用 150/200 会在高 Z+倾斜角点超限。副臂参数经角点全枚举扫描选定，
# 覆盖包络 Z∈[278,322]、ρ≤42、tilt≤18°、yaw≤25°（worst|c/m|≈0.83，见 DESIGN 2.4）。
PARAMS = {
    'R': 100.0, 'r': 50.0,      # 主臂基座 / 平台主铰分布半径
    'L': 170.0, 'l': 230.0,     # 主主动臂 / 主从动杆
    'Rac': 100.0, 'ra': 20.0,   # 副臂基座 / 平台副铰分布半径
    'La': 120.0, 'lb': 300.0,   # 副主动臂 / 副连杆
}

PHI = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)   # 方位角 0/120/240°
TWO_PI = 2.0 * math.pi

# 中性位参考电机角（首次 IK 无历史时用；主≈65° 副≈45°）
DEFAULT_REF = (1.13, 1.13, 1.13, 0.78, 0.78, 0.78)
# FK 冷启动位姿初值。姿态非零：零姿态下 d( Rot·a )/∂A = x̂×a 对 ay=0 的链恒为 0，
# 雅可比 A 列出现结构性退化，牛顿第一步即病态，故初值必须带姿态偏置。
DEFAULT_GUESS = (0.0, 0.0, 300.0, 0.09, 0.06, 0.04)


# ---------------------------------------------------------------- 位姿单位转换
def pose_deg(p):
    """弧度位姿 → 度位姿"""
    return (p[0], p[1], p[2],
            math.degrees(p[3]), math.degrees(p[4]), math.degrees(p[5]))


def pose_rad(p):
    """度位姿 → 弧度位姿"""
    return (p[0], p[1], p[2],
            math.radians(p[3]), math.radians(p[4]), math.radians(p[5]))


# ---------------------------------------------------------------- 旋转工具
def rpy_to_R(a, b, c):
    """ZYX 固定角（弧度）→ 旋转矩阵（3x3 元组，行主序）R = Rz(c)Ry(b)Rx(a)"""
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)
    return (
        (cc * cb, cc * sb * sa - sc * ca, cc * sb * ca + sc * sa),
        (sc * cb, sc * sb * sa + cc * ca, sc * sb * ca - cc * sa),
        (-sb,     cb * sa,                cb * ca),
    )


def rot_partials(ax, ay, az, a, b, c):
    """局部向量 a 经 RPY 旋转：返回 (R·a, ∂(R·a)/∂a, ∂(R·a)/∂b, ∂(R·a)/∂c)
    偏导用叉积链式展开，避免数值差分：
      ∂/∂a = Rz·Ry·(x̂ × Rx·a)
      ∂/∂b = Rz·(ŷ × Ry·Rx·a)
      ∂/∂c = ẑ × R·a
    """
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)
    # a1 = Rx(a)·v
    a1x = ax
    a1y = ca * ay - sa * az
    a1z = sa * ay + ca * az
    # a2 = Ry(b)·a1
    a2x = cb * a1x + sb * a1z
    a2y = a1y
    a2z = -sb * a1x + cb * a1z
    # Ra = Rz(c)·a2 = R·v
    rax = cc * a2x - sc * a2y
    ray = sc * a2x + cc * a2y
    raz = a2z
    # dA = Rz·Ry·(x̂ × a1)，x̂×a1 = (0, -a1z, a1y)
    t1x = sb * a1y
    t1y = -a1z
    t1z = cb * a1y
    dax = cc * t1x - sc * t1y
    day = sc * t1x + cc * t1y
    daz = t1z
    # dB = Rz·(ŷ × a2)，ŷ×a2 = (a2z, 0, -a2x)
    dbx = cc * a2z
    dby = sc * a2z
    dbz = -a2x
    # dC = ẑ × Ra = (-ray, rax, 0)
    dcx = -ray
    dcy = rax
    dcz = 0.0
    return ((rax, ray, raz), (dax, day, daz), (dbx, dby, dbz), (dcx, dcy, dcz))


# ---------------------------------------------------------------- 单链闭式解
def _adist(x, y):
    """圆周距离 [0, π]"""
    d = abs(x - y) % TWO_PI
    return d if d < math.pi else TWO_PI - d


def solve_chain(u, v, w, arm, rod, ref):
    """单链闭式解：|(u-arm·cosθ, v, w-arm·sinθ)| = rod
    返回距 ref 最近的合法解（sinθ>0，肘点在板下方），无解返回 None"""
    m2 = u * u + w * w
    if m2 < 1e-12:
        return None
    m = math.sqrt(m2)
    c = (arm * arm + m2 + v * v - rod * rod) / (2.0 * arm)
    x = c / m
    if x > 1.0 or x < -1.0:
        return None
    alpha = math.atan2(w, u)
    d = math.acos(x)
    best = None
    best_d = 0.0
    for th in (alpha + d, alpha - d):
        tw = math.fmod(th, TWO_PI)
        if tw < 0.0:
            tw += TWO_PI
        if math.sin(tw) <= 1e-9:
            continue
        dd = _adist(tw, ref % TWO_PI)
        if best is None or dd < best_d:
            best = tw
            best_d = dd
    return best


# ---------------------------------------------------------------- IK（闭式）
def ik(pose, theta_ref=DEFAULT_REF, P=None):
    """位姿 → 6 电机角（全闭式）。任一链无解返回 None。
    pose=(X,Y,Z,A,B,C) A/B/C 弧度；theta_ref 为解选择参考角"""
    P = P or PARAMS
    R, r = P['R'], P['r']
    L, l = P['L'], P['l']
    Rac, ra = P['Rac'], P['ra']
    La, lb = P['La'], P['lb']
    px, py, pz, a4, b4, c4 = pose
    out = [0.0] * 6
    for k in range(3):
        cph = math.cos(PHI[k])
        sph = math.sin(PHI[k])
        # 主链：平台铰点 = p + Rot·(r·n)
        Ra, _, _, _ = rot_partials(r * cph, r * sph, 0.0, a4, b4, c4)
        ax, ay, az = px + Ra[0], py + Ra[1], pz + Ra[2]
        u = ax * cph + ay * sph - R
        v = -ax * sph + ay * cph
        th = solve_chain(u, v, az, L, l, theta_ref[k])
        if th is None:
            return None
        out[k] = th
        # 副链：平台铰点 = p + Rot·(ra·n)
        Ra, _, _, _ = rot_partials(ra * cph, ra * sph, 0.0, a4, b4, c4)
        ax, ay, az = px + Ra[0], py + Ra[1], pz + Ra[2]
        u = ax * cph + ay * sph - Rac
        v = -ax * sph + ay * cph
        th = solve_chain(u, v, az, La, lb, theta_ref[3 + k])
        if th is None:
            return None
        out[3 + k] = th
    return tuple(out)


# ---------------------------------------------------------------- 线性求解
def _solve6(J, b):
    """高斯消元（部分主元）解 6x6。返回 (解, det)；奇异返回 (None, 0)"""
    M = [J[i][:] + [b[i]] for i in range(6)]
    det = 1.0
    for col in range(6):
        piv = col
        pmax = abs(M[col][col])
        for rr in range(col + 1, 6):
            if abs(M[rr][col]) > pmax:
                pmax = abs(M[rr][col])
                piv = rr
        if pmax < 1e-12:
            return None, 0.0
        if piv != col:
            M[col], M[piv] = M[piv], M[col]
            det = -det
        pv = M[col][col]
        det *= pv
        for rr in range(col + 1, 6):
            fct = M[rr][col] / pv
            if fct != 0.0:
                row_r = M[rr]
                row_c = M[col]
                for c2 in range(col, 7):
                    row_r[c2] -= fct * row_c[c2]
    x = [0.0] * 6
    for rr in range(5, -1, -1):
        s = M[rr][6]
        for c2 in range(rr + 1, 6):
            s -= M[rr][c2] * x[c2]
        x[rr] = s / M[rr][rr]
    return x, det


# ---------------------------------------------------------------- FK（牛顿迭代）
def fk(angles, guess=DEFAULT_GUESS, P=None, tol=1e-8, max_iter=60):
    """6 电机角 → 位姿（牛顿迭代，温启动，列归一化预处理）。
    返回 (pose6, iters, err_mm, vol)；
      err_mm = 收敛时 max||铰点距离|-杆长|（mm）
      vol = 列归一化后 |det(J)|/Π行范数 ∈ (0,1]，越接近 0 越接近奇异
        （先按列范数归一消除 mm/rad 量纲差，才是真实的奇异裕度）
    不收敛返回 None。"""
    P = P or PARAMS
    R, r = P['R'], P['r']
    L, l = P['L'], P['l']
    Rac, ra = P['Rac'], P['ra']
    La, lb = P['La'], P['lb']
    # 肘点（角度固定 → 常量）
    elbows = [None] * 6
    # 链表: (局部铰点 ax,ay, 臂长, 杆长)
    chain_loc = [None] * 6
    for k in range(3):
        cph = math.cos(PHI[k])
        sph = math.sin(PHI[k])
        th = angles[k]
        ce, se = math.cos(th), math.sin(th)
        er = R + L * ce
        elbows[k] = (er * cph, er * sph, L * se)
        chain_loc[k] = (r * cph, r * sph, L, l)
        ph = angles[3 + k]
        ce, se = math.cos(ph), math.sin(ph)
        er = Rac + La * ce
        elbows[3 + k] = (er * cph, er * sph, La * se)
        chain_loc[3 + k] = (ra * cph, ra * sph, La, lb)

    x = list(guess[:6])
    for it in range(1, max_iter + 1):
        px, py, pz = x[0], x[1], x[2]
        a4, b4, c4 = x[3], x[4], x[5]
        J = [None] * 6
        f = [0.0] * 6
        maxerr = 0.0
        for k in range(6):
            lax, lay, arm, rod = chain_loc[k]
            Ra, dA, dB, dC = rot_partials(lax, lay, 0.0, a4, b4, c4)
            ax = px + Ra[0]
            ay = py + Ra[1]
            az = pz + Ra[2]
            ex, ey, ez = elbows[k]
            dx = ax - ex
            dy = ay - ey
            dz = az - ez
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            err = abs(dist - rod)
            if err > maxerr:
                maxerr = err
            f[k] = dx * dx + dy * dy + dz * dz - rod * rod
            J[k] = [2.0 * dx, 2.0 * dy, 2.0 * dz,
                    2.0 * (dx * dA[0] + dy * dA[1] + dz * dA[2]),
                    2.0 * (dx * dB[0] + dy * dB[1] + dz * dB[2]),
                    2.0 * (dx * dC[0] + dy * dC[1] + dz * dC[2])]
        if maxerr < tol:
            # vol：列归一化后的体积比（消除 mm/rad 量纲差）
            cn = [0.0] * 6
            for c2 in range(6):
                s = 0.0
                for rr in range(6):
                    v = J[rr][c2]
                    s += v * v
                cn[c2] = math.sqrt(s)
            Jn = [[J[rr][c2] / cn[c2] for c2 in range(6)] for rr in range(6)]
            _, det = _solve6(Jn, [0.0] * 6)
            prod = 1.0
            for rr in range(6):
                s = 0.0
                for c2 in range(6):
                    v = Jn[rr][c2]
                    s += v * v
                prod *= math.sqrt(s)
            vol = abs(det) / prod if prod > 0.0 else 0.0
            return (tuple(x), it - 1, maxerr, vol)
        if maxerr > 1e9:
            return None
        # 列归一化预处理：解 (J·D⁻¹)(D·Δ) = f，改善 mm/rad 混合量纲下的条件数
        cn = [0.0] * 6
        for c2 in range(6):
            s = 0.0
            for rr in range(6):
                v = J[rr][c2]
                s += v * v
            cn[c2] = math.sqrt(s)
        Jn = [[J[rr][c2] / cn[c2] for c2 in range(6)] for rr in range(6)]
        s_n, det = _solve6(Jn, f)
        if s_n is None:
            return None
        dx6 = [s_n[c2] / cn[c2] for c2 in range(6)]
        # 步长限幅：位置 50mm / 角度 0.5rad
        step = max(abs(dx6[0]), abs(dx6[1]), abs(dx6[2])) / 50.0
        for k in range(3, 6):
            s = abs(dx6[k]) / 0.5
            if s > step:
                step = s
        if step > 1.0:
            for k in range(6):
                dx6[k] /= step
        for k in range(6):
            x[k] -= dx6[k]
    return None


# ---------------------------------------------------------------- 前馈（解析关节速度）
def ik_vel(pose, angles, twist, P=None):
    """位姿+电机角+位姿速度 → 关节速度（解析）。
    twist=(vx,vy,vz,wa,wb,wc)：线速度 mm/s + RPY 变化率 rad/s
    返回 (dθ1..dθ6) rad/s"""
    P = P or PARAMS
    px, py, pz, a4, b4, c4 = pose
    vx, vy, vz, wa, wb, wc = twist
    R, r = P['R'], P['r']
    L, l = P['L'], P['l']
    Rac, ra = P['Rac'], P['ra']
    La, lb = P['La'], P['lb']
    out = [0.0] * 6
    for k in range(3):
        cph = math.cos(PHI[k])
        sph = math.sin(PHI[k])
        th = angles[k]
        se, ce = math.sin(th), math.cos(th)
        Ra, dA, dB, dC = rot_partials(r * cph, r * sph, 0.0, a4, b4, c4)
        dx = px + Ra[0] - (R + L * ce) * cph
        dy = py + Ra[1] - (R + L * ce) * sph
        dz = pz + Ra[2] - L * se
        wx = dA[0] * wa + dB[0] * wb + dC[0] * wc
        wy = dA[1] * wa + dB[1] * wb + dC[1] * wc
        wz = dA[2] * wa + dB[2] * wb + dC[2] * wc
        num = dx * (vx + wx) + dy * (vy + wy) + dz * (vz + wz)
        den = dx * (-L * se * cph) + dy * (-L * se * sph) + dz * (L * ce)
        out[k] = num / den
        ph = angles[3 + k]
        se, ce = math.sin(ph), math.cos(ph)
        Ra, dA, dB, dC = rot_partials(ra * cph, ra * sph, 0.0, a4, b4, c4)
        dx = px + Ra[0] - (Rac + La * ce) * cph
        dy = py + Ra[1] - (Rac + La * ce) * sph
        dz = pz + Ra[2] - La * se
        wx = dA[0] * wa + dB[0] * wb + dC[0] * wc
        wy = dA[1] * wa + dB[1] * wb + dC[1] * wc
        wz = dA[2] * wa + dB[2] * wb + dC[2] * wc
        num = dx * (vx + wx) + dy * (vy + wy) + dz * (vz + wz)
        den = dx * (-La * se * cph) + dy * (-La * se * sph) + dz * (La * ce)
        out[3 + k] = num / den
    return tuple(out)
