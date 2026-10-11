"""
Delta 机器人运动控制 - 反向验证 & 奇异点检测（PLCopen 架构版）
================================================
适配新架构：
  · DeltaAxis 共享轴 + FB_PTP/FB_LIN/FB_CIR 指令FB + FB_DeltaCore 纯引擎
  · th 不重置，预处理运动与受测运动在同一 axis/sim 上连续进行
  · IK失败时验证 TarTheta 保持（不跳变）
"""
import math
from test_delta import (DeltaAxis, FB_DeltaCore, FB_PTP, FB_LIN, FB_CIR,
                        Sim, ptp, lin, cir, R, r, L, l, CYCLE)

# test_delta 未暴露 fk，这里本地实现
COS1, SIN1 = 1.0, 0.0
COS2, SIN2 = -0.5, 0.8660254037844386
COS3, SIN3 = -0.5, -0.8660254037844386


def fk(th1, th2, th3):
    e = R - r
    s1 = e + L * math.cos(th1); s2 = e + L * math.cos(th2); s3 = e + L * math.cos(th3)
    sn1 = L * math.sin(th1); sn2 = L * math.sin(th2); sn3 = L * math.sin(th3)
    a1 = -2.0*(s2*COS2-s1*COS1); b1 = -2.0*(s2*SIN2-s1*SIN1)
    c1 = -2.0*(sn2-sn1); d1 = (s1*s1+sn1*sn1)-(s2*s2+sn2*sn2)
    a2 = -2.0*(s3*COS3-s1*COS1); b2 = -2.0*(s3*SIN3-s1*SIN1)
    c2 = -2.0*(sn3-sn1); d2 = (s1*s1+sn1*sn1)-(s3*s3+sn3*sn3)
    Det = a1*b2-a2*b1
    if abs(Det) < 1e-9: return None, False
    px = (b2*d1-b1*d2)/Det; qx = (b1*c2-b2*c1)/Det
    py = (a1*d2-a2*d1)/Det; qy = (a2*c1-a1*c2)/Det
    X0 = px-s1*COS1; Y0 = py-s1*SIN1
    Az = qx*qx+qy*qy+1.0; Bz = 2.0*(X0*qx+Y0*qy)-2.0*sn1
    Cz = X0*X0+Y0*Y0+sn1*sn1-l*l; Dz = Bz*Bz-4.0*Az*Cz
    if Dz < 0: return None, False
    z1 = (-Bz+math.sqrt(Dz))/(2.0*Az); z2 = (-Bz-math.sqrt(Dz))/(2.0*Az)
    z = z1 if z1 > 0 else (z2 if z2 > 0 else z1)
    return (px+qx*z, py+qy*z, z), True


class VerifyResult:
    def __init__(self):
        self.fk_errors = []
        self.ik_fail = []
        self.fk_fail = []
        self.singularity = []
        self.theta_jump = []
        self.ik_fail_safe = 0  # IK失败时TarTheta保持安全
        self.max_fk_error = 0.0
        self.max_theta_jump = 0.0
        self.max_deform_ratio = 0.0
        self.completed = False


def _monitor(axis, res, prev_th, prev_plan, fk_thresh, theta_jump_thresh,
             deform_thresh, core=None):
    """单周期反向验证（仅在轴运动时），返回本周期后的 prev 状态"""
    if axis.Busy:
        # 1. 反向验证：FK(TarTheta) 应 ≈ Plan
        fk_pos, fk_ok = fk(axis.TarTheta1, axis.TarTheta2, axis.TarTheta3)
        if not fk_ok:
            res.fk_fail.append(0)
        else:
            plan = (axis.PlanX, axis.PlanY, axis.PlanZ)
            err = math.sqrt(sum((fk_pos[k]-plan[k])**2 for k in range(3)))
            res.max_fk_error = max(res.max_fk_error, err)
            if err > fk_thresh:
                res.fk_errors.append(err)

        # 2. IK失败检测 + 安全保持验证
        ik_ok = core.fbKin.IK_OK if core is not None else True
        if not ik_ok:
            res.ik_fail.append(0)
            dth_keep = max(abs(axis.TarTheta1-prev_th[0]),
                           abs(axis.TarTheta2-prev_th[1]),
                           abs(axis.TarTheta3-prev_th[2]))
            if dth_keep < 0.01:
                res.ik_fail_safe += 1

        # 3. 角度跳变
        dth = max(abs(axis.TarTheta1-prev_th[0]),
                  abs(axis.TarTheta2-prev_th[1]),
                  abs(axis.TarTheta3-prev_th[2]))
        res.max_theta_jump = max(res.max_theta_jump, dth)
        if dth > theta_jump_thresh:
            res.theta_jump.append(dth)

        # 4. 奇异点（形变比）
        if prev_plan is not None:
            plan = (axis.PlanX, axis.PlanY, axis.PlanZ)
            dplan = math.sqrt(sum((plan[k]-prev_plan[k])**2 for k in range(3)))
            if dplan > 1e-6:
                ratio = dth / dplan
                res.max_deform_ratio = max(res.max_deform_ratio, ratio)
                if ratio > deform_thresh:
                    res.singularity.append(ratio)

    return [axis.TarTheta1, axis.TarTheta2, axis.TarTheta3], \
           (axis.PlanX, axis.PlanY, axis.PlanZ)


def run_with_verify(axis, core, sim, fb, p, max_cycles=8000,
                    fk_thresh=0.5, theta_jump_thresh=0.05, deform_thresh=50.0):
    """驱动单个指令FB到完成并逐周期反向验证，返回 (res, cycles)"""
    res = VerifyResult()
    prev_th = [axis.TarTheta1, axis.TarTheta2, axis.TarTheta3]
    prev_plan = None

    for i in range(max_cycles):
        sim.step(core, [(fb, i < 1, p)])
        prev_th, prev_plan = _monitor(axis, res, prev_th, prev_plan,
                                      fk_thresh, theta_jump_thresh,
                                      deform_thresh, core)
        if i >= 1 and fb.InPos and not fb.Busy:
            for _ in range(20):
                sim.step(core, [(fb, False, p)])
            res.completed = True
            return res, i
    res.completed = False
    return res, -1


def print_result(name, res, cycles, check_ik=True):
    print(f"\n  --- {name} (周期{cycles}) ---")
    print(f"  FK反解最大误差: {res.max_fk_error:.4f} mm")
    print(f"  角度最大跳变:   {res.max_theta_jump:.6f} rad")
    print(f"  最大形变比:     {res.max_deform_ratio:.2f} rad/mm")
    print(f"  FK反解失败:     {len(res.fk_fail)}")
    print(f"  IK失败:         {len(res.ik_fail)} (其中安全保持={res.ik_fail_safe})")
    print(f"  FK误差超限:     {len(res.fk_errors)}")
    print(f"  角度跳变超限:   {len(res.theta_jump)}")
    print(f"  奇异点:         {len(res.singularity)}")
    ok = (len(res.fk_errors) == 0 and len(res.fk_fail) == 0 and
          len(res.theta_jump) == 0 and len(res.singularity) == 0)
    if check_ik:
        ok = ok and len(res.ik_fail) == 0
    else:
        # 允许IK失败，但要求安全保持
        ok = ok and (len(res.ik_fail) == res.ik_fail_safe)
    print(f"  结果: {'✅ 通过' if ok else '❌ 失败'}")
    return ok


total_ok = 0; total_fail = 0


def case(name, fb, p, pre=None, check_ik=True, max_cycles=8000):
    """pre=(pre_fb, pre_p)；预处理与受测运动共享同一 axis/sim，角度连续"""
    global total_ok, total_fail
    axis = DeltaAxis()
    core = FB_DeltaCore()
    sim = Sim(axis)
    sim.step(core, [])   # 先使能核心一拍
    if pre is not None:
        run_with_verify(axis, core, sim, pre[0], pre[1], max_cycles=5000)
    res, cycles = run_with_verify(axis, core, sim, fb, p, max_cycles=max_cycles)
    ok = print_result(name, res, cycles, check_ik)
    if ok: total_ok += 1
    else: total_fail += 1
    return axis, core, sim


print("=" * 70)
print("一、单运动反向验证（工作空间内目标）")
print("=" * 70)

case("PTP 梯形 短行程",
     FB_PTP(), ptp(30, 20, 310, speed=150, accel=800, move_mode=0),
     pre=(FB_PTP(), ptp(0, 0, 300, speed=200, accel=1000)))

case("PTP 梯形 长行程",
     FB_PTP(), ptp(-80, 60, 330, speed=250, accel=1500, move_mode=0),
     pre=(FB_PTP(), ptp(0, 0, 300, speed=200, accel=1000)))

case("PTP S曲线",
     FB_PTP(), ptp(50, -40, 320, speed=200, accel=1000, jerk=10000, move_mode=1),
     pre=(FB_PTP(), ptp(0, 0, 300, speed=200, accel=1000)))

case("LIN 梯形 水平",
     FB_LIN(), lin(80, 60, 300, speed=180, accel=900, move_mode=0),
     pre=(FB_PTP(), ptp(0, 0, 300, speed=200, accel=1000)))

case("LIN 梯形 垂直(工作空间内Z=330)",
     FB_LIN(), lin(0, 0, 330, speed=150, accel=800, move_mode=0),
     pre=(FB_PTP(), ptp(0, 0, 280, speed=200, accel=1000)))

case("LIN S曲线 斜线",
     FB_LIN(), lin(60, 40, 320, speed=160, accel=1000, jerk=10000, move_mode=1),
     pre=(FB_PTP(), ptp(0, 0, 300, speed=200, accel=1000)))

case("CIR 圆心式 90°",
     FB_CIR(), cir(50, 50, 300, aux=(50, 0, 300), cir_mode=0),
     pre=(FB_PTP(), ptp(100, 0, 300, speed=200, accel=1000)))

case("CIR 三点式",
     FB_CIR(), cir(0, 0, 300, aux=(50, 50, 300), cir_mode=1, move_mode=1),
     pre=(FB_PTP(), ptp(100, 0, 300, speed=200, accel=1000)))


print("\n" + "=" * 70)
print("二、BufferMode 组合（打断/缓存期间反向验证）")
print("=" * 70)

# 打断 LIN→PTP
axis = DeltaAxis(); core = FB_DeltaCore(); sim = Sim(axis)
sim.step(core, [])
fb0 = FB_PTP()
run_with_verify(axis, core, sim, fb0, ptp(0, 0, 300, speed=200, accel=1000))
fb1 = FB_LIN(); fb2 = FB_PTP()
p1 = lin(100, 0, 300, speed=50, accel=200, buffer_mode=1)
p2 = ptp(0, 80, 320, speed=200, accel=1000, buffer_mode=1)
res = VerifyResult()
prev_th = [axis.TarTheta1, axis.TarTheta2, axis.TarTheta3]
prev_plan = None
last_i = 0
for i in range(1500):
    calls = [(fb1, i == 0, p1), (fb2, False, p2)]
    if i == 200:
        calls[1] = (fb2, True, p2)
    sim.step(core, calls)
    prev_th, prev_plan = _monitor(axis, res, prev_th, prev_plan,
                                  0.5, 0.05, 50.0, core)
    last_i = i
    if i > 200 and fb2.InPos and not fb2.Busy:
        break
ok = print_result("打断: LIN→PTP", res, last_i)
total_ok += ok; total_fail += (1-ok)

# 缓存 LIN→LIN
axis = DeltaAxis(); core = FB_DeltaCore(); sim = Sim(axis)
sim.step(core, [])
fb0 = FB_PTP()
run_with_verify(axis, core, sim, fb0, ptp(0, 0, 300, speed=200, accel=1000))
fb1 = FB_LIN(); fb2 = FB_LIN()
p1 = lin(80, 0, 300, speed=100, accel=500, buffer_mode=2)
p2 = lin(80, 60, 300, speed=100, accel=500, buffer_mode=2)
res = VerifyResult()
prev_th = [axis.TarTheta1, axis.TarTheta2, axis.TarTheta3]
prev_plan = None
last_i = 0
for i in range(4000):
    calls = [(fb1, i == 0, p1), (fb2, False, p2)]
    if i == 5:
        calls[1] = (fb2, True, p2)
    sim.step(core, calls)
    # 缓存等待期不监测跳变（等待期轴仍由fb1驱动，照常监测即可）
    prev_th, prev_plan = _monitor(axis, res, prev_th, prev_plan,
                                  0.5, 0.05, 50.0, core)
    last_i = i
    if i > 10 and fb2.InPos and not fb2.Busy and abs(axis.PlanY-60) < 2:
        break
ok = print_result("缓存: LIN→LIN", res, last_i)
total_ok += ok; total_fail += (1-ok)


print("\n" + "=" * 70)
print("三、工作空间边界")
print("=" * 70)

case("PTP 接近上边界 Z=335",
     FB_PTP(), ptp(0, 0, 335, speed=150, accel=800, move_mode=0),
     pre=(FB_PTP(), ptp(0, 0, 300, speed=200, accel=1000)))

case("PTP 接近下边界 Z=210",
     FB_PTP(), ptp(0, 0, 210, speed=150, accel=800, move_mode=0),
     pre=(FB_PTP(), ptp(0, 0, 300, speed=200, accel=1000)))


print("\n" + "=" * 70)
print("四、目标不可达时的安全行为验证")
print("=" * 70)

# 4.1 PTP 目标不可达：内部IK失败，tgtTh=curTh，不运动
axis = DeltaAxis(); core = FB_DeltaCore(); sim = Sim(axis)
sim.step(core, [])
fb0 = FB_PTP()
run_with_verify(axis, core, sim, fb0, ptp(0, 0, 300, speed=200, accel=1000))
fb = FB_PTP()
p = ptp(200, 200, 500, speed=200, accel=1000)  # 工作空间外
plan_start = plan_end = None
for i in range(200):
    sim.step(core, [(fb, i == 0, p)])
    if i == 1: plan_start = (axis.PlanX, axis.PlanY, axis.PlanZ)
    if i == 100: plan_end = (axis.PlanX, axis.PlanY, axis.PlanZ)
move_dist = math.sqrt(sum((plan_end[k]-plan_start[k])**2 for k in range(3)))
print(f"  PTP不可达目标(200,200,500):")
print(f"    起点Plan={tuple(round(v,2) for v in plan_start)}, "
      f"100周期后Plan={tuple(round(v,2) for v in plan_end)}")
print(f"    移动距离={move_dist:.4f} mm")
if move_dist < 1.0:
    print("  ✅ PTP目标不可达时不运动（安全）")
    total_ok += 1
else:
    print("  ❌ PTP目标不可达时不应运动")
    total_fail += 1

# 4.2 LIN 目标不可达：规划位置超出工作空间，IK失败，TarTheta保持
axis = DeltaAxis(); core = FB_DeltaCore(); sim = Sim(axis)
sim.step(core, [])
fb0 = FB_PTP()
run_with_verify(axis, core, sim, fb0, ptp(0, 0, 300, speed=200, accel=1000))
fb = FB_LIN()
p = lin(0, 0, 400, speed=100, accel=500)  # Z=400超出上边界
prev_th = [axis.TarTheta1, axis.TarTheta2, axis.TarTheta3]
ik_fail = 0; safe_count = 0; max_jump = 0
for i in range(1500):
    sim.step(core, [(fb, i == 0, p)])
    if not core.fbKin.IK_OK:
        ik_fail += 1
        dth = max(abs(axis.TarTheta1-prev_th[0]),
                  abs(axis.TarTheta2-prev_th[1]),
                  abs(axis.TarTheta3-prev_th[2]))
        max_jump = max(max_jump, dth)
        if dth < 0.01: safe_count += 1
    prev_th = [axis.TarTheta1, axis.TarTheta2, axis.TarTheta3]
print(f"\n  LIN不可达目标(Z=400):")
print(f"    IK失败次数: {ik_fail}/1500")
print(f"    IK失败时角度最大变化: {max_jump:.6f} rad")
print(f"    IK失败时安全保持: {safe_count}/{ik_fail}")
if ik_fail > 0 and safe_count == ik_fail:
    print("  ✅ LIN目标不可达时IK失败，TarTheta保持无跳变")
    total_ok += 1
else:
    print("  ❌ LIN目标不可达时行为异常")
    total_fail += 1


print("\n" + "=" * 70)
print(f"验证结果: {total_ok} 通过, {total_fail} 失败")
print("=" * 70)
