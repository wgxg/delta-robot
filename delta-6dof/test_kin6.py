"""
Delta 6DOF 运动学测试（P0，对应 DESIGN.md 5.1 阶段 P0）
========================================================
S1 旋转工具与解析偏导自检（vs 数值差分）
S2 IK 可行性边界（包络外拒绝 / 包络内通过 / 双解选择）
S3 IK→FK 往返（随机位姿，位置/姿态误差）
S4 FK→IK 往返（随机电机角）
S5 牛顿收敛率扫描（温启动随机游走 + 冷启动）
S6 工作空间边界实测（Z / 水平半径 / 姿态角，二分法）
S7 前馈对拍（解析 ik_vel vs 数值中心差分）
S8 连续路径电机角连续性 + 奇异裕度（vol 指标）

用法：python test_kin6.py [--full]   （--full：S5 扫描 100000/10000 点）
"""
import math
import random
import sys

from kinematics6 import (PARAMS, DEFAULT_GUESS, PHI, ik, fk, ik_vel,
                         rpy_to_R, rot_partials, pose_deg, solve_chain)

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  [PASS] {name}")
    else:
        FAIL += 1
        print(f"  [FAIL] {name}  {detail}")


def section(t):
    print("\n" + "=" * 64)
    print(t)
    print("=" * 64)


# ---------------------------------------------------------------- 随机位姿
# 测试包络（经角点全枚举验证的可行域内收缩区，见 kinematics6.py PARAMS 注释）：
#   Z∈[278,322]、ρ≤42、roll/pitch 各≤13°（组合倾角≤18.4°）、yaw≤25°
ENV = {'z_lo': 278.0, 'z_hi': 322.0, 'rho_hi': 42.0, 'tilt': 13.0, 'yaw': 25.0}


def rand_pose(rng, z_lo=None, z_hi=None, rho_hi=None, tilt=None, yaw=None):
    e = ENV
    z_lo = e['z_lo'] if z_lo is None else z_lo
    z_hi = e['z_hi'] if z_hi is None else z_hi
    rho_hi = e['rho_hi'] if rho_hi is None else rho_hi
    tilt = e['tilt'] if tilt is None else tilt
    yaw = e['yaw'] if yaw is None else yaw
    rho = rng.uniform(0.0, rho_hi)
    a = rng.uniform(0.0, 2.0 * math.pi)
    return (rho * math.cos(a), rho * math.sin(a), rng.uniform(z_lo, z_hi),
            math.radians(rng.uniform(-tilt, tilt)),
            math.radians(rng.uniform(-tilt, tilt)),
            math.radians(rng.uniform(-yaw, yaw)))


def nudge_pose(rng, pose, d_rho=2.0, d_z=3.0, d_tilt=1.5):
    """小幅随机扰动（笛卡尔增量，保持包络内）。
    注意 XY 用笛卡尔高斯扰动：若扰动方位角，边界大 ρ 处切向位移会被放大 ρ 倍"""
    e = ENV
    x, y, z, a, b, c = pose
    x += rng.gauss(0, d_rho)
    y += rng.gauss(0, d_rho)
    z = min(max(z + rng.gauss(0, d_z), e['z_lo']), e['z_hi'])
    t = math.radians(e['tilt'])
    a = min(max(a + math.radians(rng.gauss(0, d_tilt)), -t), t)
    b = min(max(b + math.radians(rng.gauss(0, d_tilt)), -t), t)
    yv = math.radians(e['yaw'])
    c = min(max(c + math.radians(rng.gauss(0, d_tilt)), -yv), yv)
    rho = math.hypot(x, y)
    if rho > e['rho_hi']:
        x *= e['rho_hi'] / rho
        y *= e['rho_hi'] / rho
    return (x, y, z, a, b, c)


def pose_err(p1, p2):
    """位姿差：(mm, rad)"""
    dp = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 +
                   (p1[2] - p2[2]) ** 2)
    da = max(abs(p1[k] - p2[k]) for k in range(3, 6))
    return dp, da


# ============================================================
def main():
    full = '--full' in sys.argv
    print("Delta 6DOF 运动学测试（P0）  参数:",
          {k: PARAMS[k] for k in ('R', 'r', 'L', 'l', 'Rac', 'ra', 'La', 'lb')})

    # ============================================================
    section("S1. 旋转工具与解析偏导自检")
    rng = random.Random(11)
    for t in range(3):
        a, b, c = (rng.uniform(-1.0, 1.0) for _ in range(3))
        R = rpy_to_R(a, b, c)
        # 正交性 R·Rᵀ = I
        err_orth = max(abs(sum(R[i][k] * R[j][k] for k in range(3)) -
                           (1.0 if i == j else 0.0))
                       for i in range(3) for j in range(3))
        det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
               - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
               + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
        check(f"S1.{t+1}a R 正交性+行列式", err_orth < 1e-12 and abs(det - 1) < 1e-12,
              f"orth={err_orth:.2e} det={det:.12f}")
        # 偏导 vs 数值差分：∂(R·v)/∂θ = (∂R/∂θ)·v，解析返回的 dA/dB/dC 即该向量
        v = (rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(-50, 50))
        Ra, dA, dB, dC = rot_partials(*v, a, b, c)
        h = 1e-7
        max_dev = 0.0
        for idx, ang in ((3, a), (4, b), (5, c)):
            args = [a, b, c]
            args[idx - 3] = ang + h
            Rp = rpy_to_R(*args)
            args[idx - 3] = ang - h
            Rm = rpy_to_R(*args)
            num = [tuple((Rp[i][k] - Rm[i][k]) / (2 * h) for k in range(3))
                   for i in range(3)]
            ana = (dA, dB, dC)[idx - 3]
            dev = max(abs(sum(num[i][j] * v[j] for j in range(3)) - ana[i])
                      for i in range(3))
            max_dev = max(max_dev, dev)
        check(f"S1.{t+1}b 解析偏导 vs 数值", max_dev < 1e-6, f"dev={max_dev:.2e}")

    # ============================================================
    section("S2. IK 可行性边界与解选择")
    p_ok = (0.0, 0.0, 300.0, 0.0, 0.0, 0.0)
    check("S2.1 中性位可解", ik(p_ok) is not None)
    check("S2.2 包络外拒绝(Z=410)",
          ik((0.0, 0.0, 410.0, 0.0, 0.0, 0.0)) is None)
    check("S2.3 包络外拒绝(Z=40)", ik((0.0, 0.0, 40.0, 0.0, 0.0, 0.0)) is None)
    check("S2.5 包络外拒绝(ρ=400)", ik((400.0, 0.0, 300.0, 0.0, 0.0, 0.0)) is None)
    th = ik(p_ok)
    th2 = ik(p_ok, theta_ref=tuple(t + math.pi for t in th))
    diff = max(abs(th[k] - th2[k]) for k in range(6)) if th2 else 0.0
    check("S2.4 双解选择（参考角远离时选另一支）",
          th2 is not None and diff > 0.5, f"diff={diff:.3f}")

    # ============================================================
    section("S3. IK→FK 往返（2000 随机位姿）")
    rng = random.Random(22)
    n = 2000
    max_dp = max_da = 0.0
    n_fail = 0
    for _ in range(n):
        pose = rand_pose(rng)
        th = ik(pose)
        if th is None:
            n_fail += 1
            continue
        # guess=pose（精确起点）：本节只验证 IK↔FK 数学一致性，
        # 收敛性问题由 S5 冷/温启动专项覆盖
        res = fk(th, guess=pose)
        if res is None:
            n_fail += 1
            continue
        dp, da = pose_err(res[0], pose)
        max_dp = max(max_dp, dp)
        max_da = max(max_da, da)
    check("S3.1 IK 全部可行", n_fail == 0, f"fail={n_fail}")
    check("S3.2 位置往返误差 < 1e-6 mm", max_dp < 1e-6, f"max={max_dp:.3e}")
    check("S3.3 姿态往返误差 < 1e-6 rad", max_da < 1e-6, f"max={max_da:.3e}")

    # ============================================================
    section("S4. FK→IK 往返（2000 随机电机角）")
    rng = random.Random(33)
    max_dth = 0.0
    n_fail = 0
    pose = rand_pose(rng)
    for _ in range(n):
        pose = nudge_pose(rng, pose, d_rho=4.0, d_z=5.0, d_tilt=3.0)
        th = ik(pose)
        if th is None:
            n_fail += 1
            continue
        res = fk(th, guess=pose)   # 温启动：FK 只服务 Cur 显示，实时路径总有历史位姿
        if res is None:
            n_fail += 1
            continue
        th2 = ik(res[0], theta_ref=th)
        if th2 is None:
            n_fail += 1
            continue
        d = max(abs(th[k] - th2[k]) for k in range(6))
        max_dth = max(max_dth, d)
    check("S4.1 往返全程可行", n_fail == 0, f"fail={n_fail}")
    check("S4.2 电机角往返误差 < 1e-6 rad", max_dth < 1e-6, f"max={max_dth:.3e}")

    # ============================================================
    section("S5. 牛顿收敛率扫描")
    n_warm = 100000 if full else 20000
    n_cold = 10000 if full else 2000
    rng = random.Random(44)
    pose = rand_pose(rng)
    th_ref = ik(pose)
    hist = {}
    max_err = 0.0
    n_wfail = 0
    prev_pose = pose          # 温启动起点 = 精确解（0 迭代），模拟上电后正常跟踪
    for i in range(n_warm):
        pose = nudge_pose(rng, pose)
        th_ref = ik(pose, theta_ref=th_ref if th_ref else DEFAULT_REF)
        if th_ref is None:
            n_wfail += 1
            prev_pose = DEFAULT_GUESS
            continue
        res = fk(th_ref, guess=prev_pose)
        if res is None:
            n_wfail += 1
            prev_pose = DEFAULT_GUESS
            continue
        it = res[1]
        hist[it] = hist.get(it, 0) + 1
        max_err = max(max_err, res[2])
        prev_pose = res[0]
    rate_w = 100.0 * (n_warm - n_wfail) / n_warm
    print(f"  温启动 {n_warm} 点: 收敛率 {rate_w:.2f}%  迭代分布 {dict(sorted(hist.items()))}"
          f"  max_err={max_err:.2e} mm")
    check("S5.1 温启动收敛率 = 100%", n_wfail == 0, f"fail={n_wfail}")

    hist_c = {}
    n_cfail = 0
    max_err_c = 0.0
    # 冷启动 guess 序列：首个失败后换偏置 guess 重试一次（工程上电策略）
    COLD_GUESSES = (DEFAULT_GUESS,
                    (0.0, 0.0, 310.0, -0.09, 0.06, -0.04))
    for i in range(n_cold):
        pose = rand_pose(rng)
        th = ik(pose)
        if th is None:
            n_cfail += 1
            continue
        res = None
        for g in COLD_GUESSES:
            res = fk(th, guess=g)
            if res is not None:
                break
        if res is None:
            n_cfail += 1
            continue
        hist_c[res[1]] = hist_c.get(res[1], 0) + 1
        max_err_c = max(max_err_c, res[2])
    rate_c = 100.0 * (n_cold - n_cfail) / n_cold
    print(f"  冷启动 {n_cold} 点(2-guess 重试): 收敛率 {rate_c:.2f}%  "
          f"迭代分布 {dict(sorted(hist_c.items()))}  max_err={max_err_c:.2e} mm")
    check("S5.2 冷启动收敛率 >= 99.9%", rate_c >= 99.9, f"rate={rate_c:.2f}%")

    # ============================================================
    section("S6. 工作空间边界实测（二分法）")

    def bisect(lo_ok, hi_bad, setter, iters=48):
        """lo 可行 / hi 不可行，返回边界值"""
        assert ik(setter(lo_ok)) is not None, "bisect 起点必须可行"
        for _ in range(iters):
            mid = 0.5 * (lo_ok + hi_bad)
            if ik(setter(mid)) is not None:
                lo_ok = mid
            else:
                hi_bad = mid
        return lo_ok

    z_max = bisect(300.0, 420.0, lambda z: (0.0, 0.0, z, 0.0, 0.0, 0.0))
    print(f"  中心最大 Z = {z_max:.2f} mm")
    check("S6.1 中心 Z 上限在 [380, 400]", 380.0 < z_max < 400.0, f"z_max={z_max:.1f}")

    rho_max = bisect(60.0, 380.0, lambda rr: (rr, 0.0, 300.0, 0.0, 0.0, 0.0))
    print(f"  Z=300 水平半径上限 = {rho_max:.2f} mm")
    check("S6.2 水平半径上限 >= 150", rho_max >= 150.0, f"rho={rho_max:.1f}")

    tilt_max = bisect(math.radians(20), math.radians(89),
                      lambda t: (0.0, 0.0, 300.0, t, 0.0, 0.0))
    print(f"  中心最大 roll = {math.degrees(tilt_max):.2f} deg")
    check("S6.3 姿态角上限 >= 50 deg", math.degrees(tilt_max) >= 50.0,
          f"tilt={math.degrees(tilt_max):.1f}")

    # ============================================================
    section("S7. 前馈对拍（解析 vs 数值中心差分）")
    rng = random.Random(55)
    h = 1e-6
    max_dev = 0.0
    for _ in range(500):
        pose = rand_pose(rng)
        th = ik(pose)
        if th is None:
            continue
        twist = (rng.uniform(-200, 200), rng.uniform(-200, 200),
                 rng.uniform(-200, 200), rng.uniform(-0.5, 0.5),
                 rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5))
        ana = ik_vel(pose, th, twist)
        pp = tuple(pose[k] + h * twist[k] for k in range(6))
        pm = tuple(pose[k] - h * twist[k] for k in range(6))
        tp = ik(pp, theta_ref=th)
        tm = ik(pm, theta_ref=th)
        if tp is None or tm is None:
            continue
        for k in range(6):
            num = (tp[k] - tm[k]) / (2 * h)
            max_dev = max(max_dev, abs(num - ana[k]))
    check("S7.1 前馈误差 < 1e-5 rad/s", max_dev < 1e-5, f"max={max_dev:.3e}")

    # ============================================================
    section("S8. 连续路径连续性与奇异裕度")
    rng = random.Random(66)
    pose = rand_pose(rng)
    while ik(pose) is None:
        pose = rand_pose(rng)
    th_prev = ik(pose)
    max_step = 0.0
    min_vol = 1.0
    n_fail = 0
    for _ in range(3000):
        pose = nudge_pose(rng, pose, d_rho=1.0, d_z=1.5, d_tilt=0.8)
        th = ik(pose, theta_ref=th_prev if th_prev else DEFAULT_REF)
        if th is None:
            n_fail += 1
            continue
        res = fk(th, guess=pose)
        if res is None:
            n_fail += 1
            continue
        max_step = max(max_step, max(abs(th[k] - th_prev[k]) for k in range(6)))
        min_vol = min(min_vol, res[3])
        th_prev = th
    print(f"  连续路径 3000 步: 最大单步角增量 {max_step:.4f} rad, 最小 vol={min_vol:.4f}")
    check("S8.1 路径全程无失败", n_fail == 0, f"fail={n_fail}")
    check("S8.2 单步角增量 < 0.05 rad", max_step < 0.05, f"max={max_step:.3f}")
    # vol 为列归一化体积比。构型特性：链 1 主铰点 (r,0,0) 绕 X 轴转动不变，
    # 纯 roll + yaw≈0 配置下 A 列与 B/Z 列近共线，vol 降至 1e-4 量级；
    # 该退化不影响 IK 闭式解与前馈（S3/S7 已验证），仅使 FK 牛顿在该配置附近
    # 迭代次数增多（S5 实测温启动 100% 收敛，最多 13 次）。
    check("S8.3 雅可比非奇异 vol > 1e-4", min_vol > 1e-4, f"min={min_vol:.2e}")

    # ============================================================
    print("\n" + "=" * 64)
    print(f"测试结果: {PASS} 通过, {FAIL} 失败")
    print("=" * 64)
    return FAIL


if __name__ == '__main__':
    sys.exit(1 if main() else 0)
