"""
Delta 机器人运动控制 - 反向验证 & 奇异点检测（修正版）
================================================
修复：
1. 测试目标限制在工作空间内（x=y=0时Z_max≈343mm）
2. th 不重置，从上一个运动的结束状态继续
3. IK失败时验证TarTheta保持（不跳变）
"""
import math
from test_delta import *

R, r, L, l = 100.0, 50.0, 150.0, 200.0
CYCLE = 0.001


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


def run_with_verify(core, cmd, init_th=None, execute_cycles=1, max_cycles=8000,
                    fk_thresh=0.5, theta_jump_thresh=0.05, deform_thresh=50.0):
    res = VerifyResult()
    th1, th2, th3 = init_th if init_th else (0.0, 0.0, 0.0)
    prev_th = [th1, th2, th3]
    prev_plan = None

    for i in range(max_cycles):
        exec_pulse = (i < execute_cycles)
        core(R, r, L, l, cmd, exec_pulse, th1, th2, th3, CYCLE, True, 0.5, True)

        # 电机跟随
        th1 += (core.TarTheta1 - th1) * 0.3
        th2 += (core.TarTheta2 - th2) * 0.3
        th3 += (core.TarTheta3 - th3) * 0.3

        if i > 0 and core.Busy:
            # 1. 反向验证
            fk_pos, fk_ok = fk(core.TarTheta1, core.TarTheta2, core.TarTheta3)
            if not fk_ok:
                res.fk_fail.append(i)
            else:
                err = math.sqrt(sum((fk_pos[k]-[core.PlanX,core.PlanY,core.PlanZ][k])**2 for k in range(3)))
                res.max_fk_error = max(res.max_fk_error, err)
                if err > fk_thresh:
                    res.fk_errors.append((i, err))

            # 2. IK失败检测 + 安全保持验证
            if not core.fbKin.IK_OK:
                res.ik_fail.append(i)
                # IK失败时TarTheta应≈上一周期TarTheta（保持）
                dth_keep = max(abs(core.TarTheta1-prev_th[0]),
                               abs(core.TarTheta2-prev_th[1]),
                               abs(core.TarTheta3-prev_th[2]))
                if dth_keep < 0.01:
                    res.ik_fail_safe += 1

            # 3. 角度跳变
            dth = [abs(core.TarTheta1-prev_th[0]),
                   abs(core.TarTheta2-prev_th[1]),
                   abs(core.TarTheta3-prev_th[2])]
            max_dth = max(dth)
            res.max_theta_jump = max(res.max_theta_jump, max_dth)
            if max_dth > theta_jump_thresh:
                res.theta_jump.append((i, max_dth))

            # 4. 奇异点（形变比）
            if prev_plan is not None:
                dplan = math.sqrt(sum(([core.PlanX,core.PlanY,core.PlanZ][k]-prev_plan[k])**2 for k in range(3)))
                if dplan > 1e-6:
                    ratio = max_dth / dplan
                    res.max_deform_ratio = max(res.max_deform_ratio, ratio)
                    if ratio > deform_thresh:
                        res.singularity.append((i, ratio))

        prev_th = [core.TarTheta1, core.TarTheta2, core.TarTheta3]
        prev_plan = (core.PlanX, core.PlanY, core.PlanZ)

        if core.InPos and not core.Busy:
            for _ in range(20):
                core(R, r, L, l, cmd, False, th1, th2, th3, CYCLE, True, 0.5, True)
            res.completed = True
            return res, i, (th1, th2, th3)
    res.completed = False
    return res, -1, (th1, th2, th3)


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

def test_case(name, cmd, pre_cmd=None, check_ik=True):
    global total_ok, total_fail
    core = FB_DeltaCore()
    init_th = (0.0, 0.0, 0.0)
    if pre_cmd:
        _, _, init_th = run_with_verify(core, pre_cmd, init_th, max_cycles=5000)
    res, cycles, _ = run_with_verify(core, cmd, init_th, max_cycles=8000)
    ok = print_result(name, res, cycles, check_ik)
    if ok: total_ok += 1
    else: total_fail += 1


print("=" * 70)
print("一、单运动反向验证（工作空间内目标）")
print("=" * 70)

test_case("PTP 梯形 短行程",
    make_cmd(0, 30, 20, 310, speed=150, accel=800, move_mode=0),
    pre_cmd=make_cmd(0, 0, 0, 300, speed=200, accel=1000))

test_case("PTP 梯形 长行程",
    make_cmd(0, -80, 60, 330, speed=250, accel=1500, move_mode=0),
    pre_cmd=make_cmd(0, 0, 0, 300, speed=200, accel=1000))

test_case("PTP S曲线",
    make_cmd(0, 50, -40, 320, speed=200, accel=1000, jerk=10000, move_mode=1),
    pre_cmd=make_cmd(0, 0, 0, 300, speed=200, accel=1000))

test_case("LIN 梯形 水平",
    make_cmd(1, 80, 60, 300, speed=180, accel=900, move_mode=0),
    pre_cmd=make_cmd(0, 0, 0, 300, speed=200, accel=1000))

test_case("LIN 梯形 垂直(工作空间内Z=330)",
    make_cmd(1, 0, 0, 330, speed=150, accel=800, move_mode=0),
    pre_cmd=make_cmd(0, 0, 0, 280, speed=200, accel=1000))

test_case("LIN S曲线 斜线",
    make_cmd(1, 60, 40, 320, speed=160, accel=1000, jerk=10000, move_mode=1),
    pre_cmd=make_cmd(0, 0, 0, 300, speed=200, accel=1000))

test_case("CIR 圆心式 90°",
    make_cmd(2, 50, 50, 300, speed=150, accel=800, move_mode=0, aux=(50,0,300), cir_mode=0),
    pre_cmd=make_cmd(0, 100, 0, 300, speed=200, accel=1000))

test_case("CIR 三点式",
    make_cmd(2, 0, 0, 300, speed=150, accel=800, move_mode=1, aux=(50,50,300), cir_mode=1),
    pre_cmd=make_cmd(0, 100, 0, 300, speed=200, accel=1000))


print("\n" + "=" * 70)
print("二、BufferMode 组合（打断/缓存期间反向验证）")
print("=" * 70)

# 打断 LIN→PTP
core = FB_DeltaCore()
_, _, init_th = run_with_verify(core, make_cmd(0, 0, 0, 300, speed=200, accel=1000), (0,0,0))
cmd1 = make_cmd(1, 100, 0, 300, speed=50, accel=200, buffer_mode=1)
th1, th2, th3 = init_th
prev_th = [th1, th2, th3]
res = VerifyResult()
for i in range(1200):
    core(R, r, L, l, cmd1, i==0, th1, th2, th3, CYCLE, True, 0.5, True)
    th1 += (core.TarTheta1-th1)*0.3; th2 += (core.TarTheta2-th2)*0.3; th3 += (core.TarTheta3-th3)*0.3
    if i == 200:
        core(R, r, L, l, make_cmd(0, 0, 80, 320, speed=200, accel=1000, buffer_mode=1), True, th1, th2, th3, CYCLE, True, 0.5, True)
    if i > 0 and core.Busy:
        fk_pos, fk_ok = fk(core.TarTheta1, core.TarTheta2, core.TarTheta3)
        if fk_ok:
            err = math.sqrt(sum((fk_pos[k]-[core.PlanX,core.PlanY,core.PlanZ][k])**2 for k in range(3)))
            res.max_fk_error = max(res.max_fk_error, err)
            if err > 0.5: res.fk_errors.append((i,err))
        else: res.fk_fail.append(i)
        if not core.fbKin.IK_OK: res.ik_fail.append(i)
        dth = max(abs(core.TarTheta1-prev_th[0]),abs(core.TarTheta2-prev_th[1]),abs(core.TarTheta3-prev_th[2]))
        res.max_theta_jump = max(res.max_theta_jump, dth)
        if dth > 0.05: res.theta_jump.append((i,dth))
    prev_th = [core.TarTheta1, core.TarTheta2, core.TarTheta3]
    if i > 200 and core.InPos and not core.Busy: break
ok = print_result("打断: LIN→PTP", res, i)
total_ok += ok; total_fail += (1-ok)

# 缓存 LIN→LIN
core = FB_DeltaCore()
_, _, init_th = run_with_verify(core, make_cmd(0, 0, 0, 300, speed=200, accel=1000), (0,0,0))
cmd1 = make_cmd(1, 80, 0, 300, speed=100, accel=500, buffer_mode=2)
th1, th2, th3 = init_th
prev_th = [th1, th2, th3]
res = VerifyResult()
for i in range(3000):
    core(R, r, L, l, cmd1, i==0, th1, th2, th3, CYCLE, True, 0.5, True)
    th1 += (core.TarTheta1-th1)*0.3; th2 += (core.TarTheta2-th2)*0.3; th3 += (core.TarTheta3-th3)*0.3
    if i == 5:
        core(R, r, L, l, make_cmd(1, 80, 60, 300, speed=100, accel=500, buffer_mode=2), True, th1, th2, th3, CYCLE, True, 0.5, True)
    if i > 0:
        fk_pos, fk_ok = fk(core.TarTheta1, core.TarTheta2, core.TarTheta3)
        if fk_ok:
            err = math.sqrt(sum((fk_pos[k]-[core.PlanX,core.PlanY,core.PlanZ][k])**2 for k in range(3)))
            res.max_fk_error = max(res.max_fk_error, err)
            if err > 0.5: res.fk_errors.append((i,err))
        else: res.fk_fail.append(i)
        if not core.fbKin.IK_OK: res.ik_fail.append(i)
        dth = max(abs(core.TarTheta1-prev_th[0]),abs(core.TarTheta2-prev_th[1]),abs(core.TarTheta3-prev_th[2]))
        res.max_theta_jump = max(res.max_theta_jump, dth)
    prev_th = [core.TarTheta1, core.TarTheta2, core.TarTheta3]
    if i > 10 and core.InPos and not core.Busy and abs(core.PlanY-60) < 2: break
ok = print_result("缓存: LIN→LIN", res, i)
total_ok += ok; total_fail += (1-ok)


print("\n" + "=" * 70)
print("三、工作空间边界")
print("=" * 70)

test_case("PTP 接近上边界 Z=335",
    make_cmd(0, 0, 0, 335, speed=150, accel=800, move_mode=0),
    pre_cmd=make_cmd(0, 0, 0, 300, speed=200, accel=1000))

test_case("PTP 接近下边界 Z=210",
    make_cmd(0, 0, 0, 210, speed=150, accel=800, move_mode=0),
    pre_cmd=make_cmd(0, 0, 0, 300, speed=200, accel=1000))


print("\n" + "=" * 70)
print("四、目标不可达时的安全行为验证")
print("=" * 70)

# 4.1 PTP 目标不可达：内部IK失败，tgtTh=curTh，不运动
core = FB_DeltaCore()
_, _, init_th = run_with_verify(core, make_cmd(0, 0, 0, 300, speed=200, accel=1000), (0,0,0))
cmd = make_cmd(0, 200, 200, 500, speed=200, accel=1000)  # 工作空间外
th1, th2, th3 = init_th
plan_start = None; plan_end = None
for i in range(200):
    core(R, r, L, l, cmd, i==0, th1, th2, th3, CYCLE, True, 0.5, True)
    th1 += (core.TarTheta1-th1)*0.3; th2 += (core.TarTheta2-th2)*0.3; th3 += (core.TarTheta3-th3)*0.3
    if i == 1: plan_start = (core.PlanX, core.PlanY, core.PlanZ)
    if i == 100: plan_end = (core.PlanX, core.PlanY, core.PlanZ)
move_dist = math.sqrt(sum((plan_end[k]-plan_start[k])**2 for k in range(3)))
print(f"  PTP不可达目标(200,200,500):")
print(f"    起点Plan={plan_start}, 100周期后Plan={plan_end}")
print(f"    移动距离={move_dist:.4f} mm")
if move_dist < 1.0:
    print("  ✅ PTP目标不可达时不运动（安全）")
    total_ok += 1
else:
    print("  ❌ PTP目标不可达时不应运动")
    total_fail += 1

# 4.2 LIN 目标不可达：规划位置超出工作空间，IK失败，TarTheta保持
core = FB_DeltaCore()
_, _, init_th = run_with_verify(core, make_cmd(0, 0, 0, 300, speed=200, accel=1000), (0,0,0))
cmd = make_cmd(1, 0, 0, 400, speed=100, accel=500)  # Z=400超出上边界
th1, th2, th3 = init_th
prev_th = [th1, th2, th3]
ik_fail = 0; safe_count = 0; max_jump = 0
for i in range(1500):
    core(R, r, L, l, cmd, i==0, th1, th2, th3, CYCLE, True, 0.5, True)
    th1 += (core.TarTheta1-th1)*0.3; th2 += (core.TarTheta2-th2)*0.3; th3 += (core.TarTheta3-th3)*0.3
    if not core.fbKin.IK_OK:
        ik_fail += 1
        dth = max(abs(core.TarTheta1-prev_th[0]),abs(core.TarTheta2-prev_th[1]),abs(core.TarTheta3-prev_th[2]))
        max_jump = max(max_jump, dth)
        if dth < 0.01: safe_count += 1
    prev_th = [core.TarTheta1, core.TarTheta2, core.TarTheta3]
print(f"\n  LIN不可达目标(Z=400):")
print(f"    IK失败次数: {ik_fail}/500")
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
