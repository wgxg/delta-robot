"""
Delta 6DOF - 大规模多样本矩阵测试（P1）
=========================================
框架逐字移植自 3DOF test_matrix.py（调度记账模型原样适用），差异仅在几何层：
  A. 指令×打断 全交叉矩阵（PTP/LIN/CIR × PTP/LIN/CIR，梯形+S曲线）
  B. 打断时序：加速段 / 匀速段 / 减速段
  C. 拒绝矩阵 + 空闲拒绝模式正常执行
  D. 缓存组合：跨类型缓存、槽满、缓存等待者被打断、等待者自我升级、链式缓存
  E. 同实例重触发（自打断）
  F. 信号语义：InPos/CommandAborted 锁存、Rejected 单周期脉冲、
     Busy 含等待期、Execute 长电平不重触发、非法参数
  G. 失能恢复：运动中核心失能→指令复位→重新使能可运行
  H. 随机模糊测试：20 组随机指令序列（死锁/跳变/FK一致性/owner一致性/终态闭环）

目标点全部在 P0 验证包络内侧生成（Z 280~320、ρ≤40、tilt≤12°、yaw≤22°）。
"""
import math
import random

from kinematics6 import ik, fk, pose_rad
from test_delta6 import (DeltaAxis6, FB_DeltaCore6, FB_PTP6, FB_LIN6, FB_CIR6,
                         Sim6, ptp6, lin6, cir6, HOME_DEG,
                         FB_DeltaKinematics6)

# ============================================================
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


FB_CLASSES = {'PTP': FB_PTP6, 'LIN': FB_LIN6, 'CIR': FB_CIR6}


# ============================================================
# World6：多 FB 实例调度器（移植 3DOF World，6 电机 + 姿态）
# ============================================================
class World6:
    def __init__(self, ff=True, kff=0.5):
        self.axis = DeltaAxis6()
        self.core = FB_DeltaCore6()
        self.sim = Sim6(self.axis)
        self.fbs = []
        self.pulse = set()
        self.hold = set()
        self.max_jump = 0.0
        self.max_fkerr_pos = 0.0
        self.max_fkerr_ang = 0.0
        self.t = 0
        home_ang = ik(pose_rad(HOME_DEG))
        self._prev_tar = tuple(home_ang)
        self._prev_plan = HOME_DEG
        self._plan_at_tar = HOME_DEG
        self.ff, self.kff = ff, kff
        self.sim.step(self.core, [])    # 先使能一拍

    def new_fb(self, kind):
        fb = FB_CLASSES[kind]()
        self.fbs.append([fb, kind, None])
        return fb

    def trigger(self, fb, p):
        for rec in self.fbs:
            if rec[0] is fb:
                rec[2] = p
        self.pulse.add(id(fb))

    def hold_execute(self, fb):
        self.hold.add(id(fb))

    def find(self, fb):
        for rec in self.fbs:
            if rec[0] is fb:
                return rec

    def tick(self, enable=True):
        for k in range(6):
            self.axis.CurTheta[k] = self.sim.th[k]
        for rec in self.fbs:
            fb, kind, p = rec
            if p is None:
                continue
            ex = (id(fb) in self.pulse) or (id(fb) in self.hold)
            fb(self.axis, Execute=ex, **p)
        self.pulse.clear(); self.hold.clear()
        self.core(self.axis, Enable=enable,
                  FfEnable=self.ff if enable else False,
                  Kff=self.kff if enable else 0.0)
        tar = tuple(self.axis.TarTheta)
        for k in range(6):
            self.sim.th[k] += (tar[k]-self.sim.th[k])*0.3
        jump = max(abs(tar[k]-self._prev_tar[k]) for k in range(6))
        self.max_jump = max(self.max_jump, jump)
        plan = (self.axis.PlanX, self.axis.PlanY, self.axis.PlanZ,
                self.axis.PlanA, self.axis.PlanB, self.axis.PlanC)
        tar_held = all(abs(tar[k]-self._prev_tar[k]) < 1e-12 for k in range(6))
        if not tar_held:
            self._plan_at_tar = plan
        # IK 失败安全保持时 Tar 不动而 Plan 继续：累计漂移过大则不参与一致性统计
        hold_drift = math.sqrt(
            sum((plan[k]-self._plan_at_tar[k])**2 for k in range(3)) +
            sum(math.radians(plan[3+k]-self._plan_at_tar[3+k])**2 for k in range(3)))
        if self.axis.Busy and not (tar_held and hold_drift > 0.5):
            res = fk(tar, guess=pose_rad(plan))
            if res is not None:
                q = res[0]
                pr = pose_rad(plan)
                errp = math.sqrt(sum((q[k]-pr[k])**2 for k in range(3)))
                erra = max(abs(q[k]-pr[k]) for k in range(3, 6))
                self.max_fkerr_pos = max(self.max_fkerr_pos, errp)
                self.max_fkerr_ang = max(self.max_fkerr_ang, erra)
        self._prev_tar = tar
        self._prev_plan = plan
        self.t += 1

    def run_until_idle(self, timeout=6000):
        for _ in range(timeout):
            self.tick()
            if not self.axis.Busy and not self.axis.hasBuf:
                return True
        return False

    def owner_consistent(self):
        if not self.axis.Busy:
            return self.axis.owner == 0 and self.axis.ActiveType == 0
        for rec in self.fbs:
            fb = rec[0]
            if getattr(fb, 'running', False) and fb.myId == self.axis.owner:
                return rec[1] == ('PTP', 'LIN', 'CIR')[self.axis.ActiveType-1]
        return False


# ============================================================
# 可达目标点（严格在 P0 包络内侧）
# ============================================================
ENV6 = dict(z_lo=280.0, z_hi=320.0, rho_hi=40.0, tilt=12.0, yaw=22.0)


def reachable6(rng):
    rho = rng.uniform(0, ENV6['rho_hi'])
    a = rng.uniform(0, 2*math.pi)
    return (rho*math.cos(a), rho*math.sin(a),
            rng.uniform(ENV6['z_lo'], ENV6['z_hi']),
            rng.uniform(-ENV6['tilt'], ENV6['tilt']),
            rng.uniform(-ENV6['tilt'], ENV6['tilt']),
            rng.uniform(-ENV6['yaw'], ENV6['yaw']))


def cir_params6(rng, sx, sy, sz, sa=0.0, sb=0.0, sc=0.0):
    """以起点为圆弧起点生成可达的圆心式/三点式参数（圆心偏向内侧，弧不外扩）。
    姿态取绝对值较小的随机 RPY（≤10/10/18°），slerp 沿程不超出端点倾角。"""
    base = math.hypot(sx, sy)
    inward = math.atan2(sy, sx) + math.pi if base > 1e-6 else rng.uniform(0, 2*math.pi)
    ang = inward + rng.uniform(-math.pi/2, math.pi/2)
    rad = rng.uniform(8, 16)
    span = rng.choice([60, 90, 120, -60, -90, -120]) * math.pi/180
    cx, cy = sx+rad*math.cos(ang), sy+rad*math.sin(ang)
    dx, dy = sx-cx, sy-cy
    ex = cx + dx*math.cos(span) - dy*math.sin(span)
    ey = cy + dx*math.sin(span) + dy*math.cos(span)
    ez = sz
    cm_x = cx + dx*math.cos(span/2) - dy*math.sin(span/2)
    cm_y = cy + dx*math.sin(span/2) + dy*math.cos(span/2)
    a = rng.uniform(-10, 10); b = rng.uniform(-10, 10); c = rng.uniform(-18, 18)
    kw = dict(speed=rng.uniform(100, 250), accel=rng.uniform(600, 1500),
              jerk=8000, move_mode=rng.randint(0, 1))
    mode = rng.randint(0, 1)
    if mode == 0:
        return cir6(ex, ey, ez, aux=(cx, cy, sz), cir_mode=0,
                    a=a, b=b, c=c, **kw)
    return cir6(ex, ey, ez, aux=(cm_x, cm_y, ez), cir_mode=1,
                a=a, b=b, c=c, **kw)


def cmd_for6(kind, p6, rng=None, **kw):
    x, y, z, a, b, c = p6
    if kind == 'PTP':
        return ptp6(x, y, z, a, b, c, **kw)
    if kind == 'LIN':
        return lin6(x, y, z, a, b, c, **kw)
    return cir6(x, y, z, aux=kw.pop('aux', (x, y-15, z)),
                cir_mode=kw.pop('cir_mode', 0), a=a, b=b, c=c, **kw)


def killer_cir6(w, rng=None):
    """打断时刻按当前位姿程序化构造向内 90° 圆弧（保证弧在包络内）"""
    sx, sy, sz = w.axis.CurX, w.axis.CurY, w.axis.CurZ
    base = math.hypot(sx, sy)
    inward = math.atan2(sy, sx) + math.pi if base > 1e-6 else 0.0
    rad = 15.0
    cx, cy = sx+rad*math.cos(inward), sy+rad*math.sin(inward)
    dx, dy = sx-cx, sy-cy
    ex = cx + dx*math.cos(math.pi/2) - dy*math.sin(math.pi/2)
    ey = cy + dx*math.sin(math.pi/2) + dy*math.cos(math.pi/2)
    return cir6(ex, ey, sz, aux=(cx, cy, sz), cir_mode=0,
                speed=200, accel=1000, buffer_mode=1)


# ============================================================
def main():
    # ============================================================
    section("A. 打断交叉矩阵（被打断指令 × 打断指令，含梯形/S曲线）")

    for victim in ('PTP', 'LIN', 'CIR'):
        for killer in ('CIR', 'LIN', 'PTP'):
            for mm in (0, 1):
                w = World6()
                f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300))
                w.run_until_idle()
                fv = w.new_fb(victim)
                if victim == 'CIR':
                    # 起点(0,0) 圆心(20,0) 半径20 +90° → (20,20)
                    vp = cir6(20, 20, 300, aux=(20, 0, 300), cir_mode=0,
                              speed=30, accel=150, move_mode=mm, buffer_mode=1)
                elif victim == 'PTP':
                    vp = ptp6(30, 20, 310, speed=35, accel=180,
                              move_mode=mm, buffer_mode=1)
                else:
                    vp = lin6(40, 0, 300, speed=30, accel=150,
                              move_mode=mm, buffer_mode=1)
                w.trigger(fv, vp)
                fk = w.new_fb(killer)
                aborted = False
                done = False
                kp = None
                for i in range(3000):
                    if i == 120:
                        if killer == 'CIR':
                            kp = killer_cir6(w)
                        elif killer == 'PTP':
                            kp = ptp6(0, 30, 315, b=5, speed=200, accel=1000,
                                      move_mode=mm, buffer_mode=1)
                        else:
                            kp = lin6(-25, 18, 300, a=0, b=0, c=5,
                                      speed=200, accel=1000,
                                      move_mode=mm, buffer_mode=1)
                        w.trigger(fk, kp)
                    w.tick()
                    if i >= 121 and fv.CommandAborted:
                        aborted = True
                    if i > 120 and fk.InPos and not fk.Busy:
                        done = True
                        break
                tag = f"{victim}<-{killer} {'S' if mm else 'T'}"
                check(f"{tag}: 受害者 CommandAborted", aborted)
                check(f"{tag}: 打断者完成", done)
                check(f"{tag}: 令牌一致/轴空闲", w.owner_consistent() and not w.axis.Busy)

    # ============================================================
    section("B. 打断时序（慢LIN：加速段/匀速段/减速段）")
    # d=40, v=30, a=150 → 加速200周期(3mm)，匀速约1133周期
    for phase_name, hit in (("加速段", 100), ("匀速段", 600), ("减速段", 1400)):
        w = World6()
        f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
        fv = w.new_fb('LIN')
        w.trigger(fv, lin6(40, 0, 300, speed=30, accel=150, buffer_mode=1))
        fk = w.new_fb('PTP')
        ok_abort = ok_done = False
        for i in range(3000):
            if i == hit:
                w.trigger(fk, ptp6(0, 0, 315, speed=200, accel=1000, buffer_mode=1))
            w.tick()
            if i >= hit+1 and fv.CommandAborted:
                ok_abort = True
            if i > hit and fk.InPos and not fk.Busy:
                ok_done = True
                break
        check(f"{phase_name}打断: 受害者abort", ok_abort)
        check(f"{phase_name}打断: 新指令到达Z≈315",
              ok_done and abs(w.axis.PlanZ-315) < 1.5, f"Z={w.axis.PlanZ:.1f}")
        check(f"{phase_name}打断: 角度跳变有界", w.max_jump < 0.05,
              f"jump={w.max_jump:.4f}")

    # ============================================================
    section("C. 拒绝模式（运动中来任何指令都拒绝，原指令不受影响）")

    for victim in ('PTP', 'LIN', 'CIR'):
        w = World6()
        f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
        fv = w.new_fb(victim)
        if victim == 'CIR':
            vp = cir6(20, 20, 300, aux=(20, 0, 300), speed=30, accel=150,
                      buffer_mode=0)
        else:
            vp = cmd_for6(victim, (40, 0, 300, 0, 0, 0),
                          speed=35, accel=180, buffer_mode=0)
        w.trigger(fv, vp)
        rej_results = {}
        for intr in ('PTP', 'LIN', 'CIR'):
            fi = w.new_fb(intr)
            if intr == 'CIR':
                ip = cir6(-15, 15, 300, aux=(-15, -10, 300), cir_mode=1,
                          buffer_mode=0)
            else:
                ip = cmd_for6(intr, (-25, 20, 315, 5, 0, -5), buffer_mode=0)
            for i in range(1, 250):
                if i == 60:
                    w.trigger(fi, ip)
                w.tick()
                if i == 60:
                    rej_results[intr] = fi.Rejected
                if i == 61:
                    rej_results[intr+'_pulse'] = not fi.Rejected
        for intr in ('PTP', 'LIN', 'CIR'):
            check(f"{victim}运动中拒绝{intr}", rej_results.get(intr, False))
            check(f"{intr} Rejected为单周期脉冲",
                  rej_results.get(intr+'_pulse', False))
        check(f"{victim}未受影响仍在运动", fv.Busy)
        check("ActiveType 与受害者一致",
              w.axis.ActiveType == {'PTP': 1, 'LIN': 2, 'CIR': 3}[victim])

    w = World6()
    f = w.new_fb('PTP')
    w.trigger(f, ptp6(25, -20, 310, buffer_mode=0))
    idle_ok = w.run_until_idle()
    check("空闲时拒绝模式指令正常执行",
          idle_ok and f.InPos and abs(w.axis.PlanX-25) < 1)

    # ============================================================
    section("D1. 跨类型缓存（受害者×缓存等待者）")

    victim_end = {'PTP': (30, 20, 310), 'LIN': (40, 0, 300),
                  'CIR': (20, 20, 300)}
    for victim in ('PTP', 'LIN', 'CIR'):
        for waiter in ('PTP', 'LIN', 'CIR'):
            w = World6()
            f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
            fv = w.new_fb(victim)
            if victim == 'CIR':
                vp = cir6(20, 20, 300, aux=(20, 0, 300), speed=80, accel=400,
                          buffer_mode=2)
            else:
                vp = cmd_for6(victim, victim_end[victim] + (0, 0, 0),
                              speed=70, accel=350, buffer_mode=2)
            w.trigger(fv, vp)
            fw = w.new_fb(waiter)
            if waiter == 'CIR':
                sx, sy, sz = victim_end[victim]
                ccx, ccy, ccz = sx, sy-18.0, sz
                ex, ey = ccx-18.0, ccy
                wp = cir6(ex, ey, ccz, aux=(ccx, ccy, ccz), cir_mode=0,
                          speed=80, accel=400, buffer_mode=2)
                target = (ex, ey, ccz, 0, 0, 0)
            else:
                target = (-25, 20, 310, 5, -5, 8)
                wp = cmd_for6(waiter, target, speed=80, accel=400,
                              buffer_mode=2)
            waiting_seen = False
            for i in range(30):
                if i == 5:
                    w.trigger(fw, wp)
                w.tick()
                if i >= 6 and fw.Busy and not fw.running:
                    waiting_seen = True
            ok = w.run_until_idle(6000)
            tag = f"{victim}→缓存{waiter}"
            check(f"{tag}: 等待期Busy=TRUE", waiting_seen)
            check(f"{tag}: 等待者最终完成", ok and fw.InPos and not fw.Busy,
                  f"InPos={fw.InPos} Busy={fw.Busy}")
            check(f"{tag}: 到达等待者目标",
                  abs(w.axis.PlanX-target[0]) < 2 and
                  abs(w.axis.PlanY-target[1]) < 2 and
                  abs(w.axis.PlanZ-target[2]) < 2 and
                  abs(w.axis.PlanA-target[3]) < 1.5 and
                  abs(w.axis.PlanB-target[4]) < 1.5 and
                  abs(w.axis.PlanC-target[5]) < 1.5,
                  f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f},{w.axis.PlanZ:.1f},"
                  f"{w.axis.PlanA:.1f},{w.axis.PlanB:.1f},{w.axis.PlanC:.1f})")
            check(f"{tag}: 受害者不报告abort", not fv.CommandAborted)

    section("D2. 缓存等待期间：再来指令（拒绝/槽满/打断等待者）")

    # D2.1 等待期来拒绝模式
    w = World6()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin6(40, 0, 300, speed=30, accel=150,
                                             buffer_mode=2))
    fw = w.new_fb('LIN'); w.trigger(fw, lin6(40, 25, 300, speed=70, accel=350,
                                             buffer_mode=2))
    for _ in range(5):
        w.tick()
    fx = w.new_fb('PTP')
    rej = False
    for i in range(4000):
        if i == 0:
            w.trigger(fx, ptp6(0, 0, 315, buffer_mode=0))
        w.tick()
        if i == 0:
            rej = fx.Rejected
        if fw.InPos and not fw.Busy:
            break
    check("等待期拒绝模式被Rejected", rej)
    check("等待链仍正常完成", fw.InPos and abs(w.axis.PlanY-25) < 2)
    check("插队者未执行",
          not fx.InPos and not fx.running and abs(w.axis.PlanZ-315) > 5)

    # D2.2 等待期来缓存模式 → 槽满
    w = World6()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin6(40, 0, 300, speed=30, accel=150,
                                             buffer_mode=2))
    fw = w.new_fb('LIN'); w.trigger(fw, lin6(40, 25, 300, speed=70, accel=350,
                                             buffer_mode=2))
    for _ in range(5):
        w.tick()
    fx = w.new_fb('CIR')
    xp = cir6(-15, 15, 300, aux=(-15, -10, 300), cir_mode=1, buffer_mode=2)
    w.trigger(fx, xp)
    w.tick()
    check("等待期第二条缓存槽满Rejected", fx.Rejected)

    # D2.3 等待期来打断
    w = World6()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin6(40, 0, 300, speed=30, accel=150,
                                             buffer_mode=2))
    fw = w.new_fb('LIN'); w.trigger(fw, lin6(40, 25, 300, speed=70, accel=350,
                                             buffer_mode=2))
    for _ in range(5):
        w.tick()
    fx = w.new_fb('PTP'); w.trigger(fx, ptp6(-25, -25, 315, a=5, b=-5,
                                             speed=200, accel=1000,
                                             buffer_mode=1))
    w_abort = False
    for i in range(3000):
        w.tick()
        if fw.CommandAborted:
            w_abort = True
        if fx.InPos and not fx.Busy:
            break
    check("等待者被打断时CommandAborted", w_abort)
    check("打断后hasBuf清空", not w.axis.hasBuf)
    check("插队指令完成到达(-25,-25,315)",
          fx.InPos and abs(w.axis.PlanX+25) < 2 and abs(w.axis.PlanY+25) < 2
          and abs(w.axis.PlanZ-315) < 2 and abs(w.axis.PlanA-5) < 1.5,
          f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f},{w.axis.PlanZ:.1f})")

    # D2.4 等待者同实例改发打断 → 放弃等待立即执行
    w = World6()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin6(40, 0, 300, speed=30, accel=150,
                                             buffer_mode=2))
    fw = w.new_fb('LIN'); w.trigger(fw, lin6(40, 25, 300, speed=70, accel=350,
                                             buffer_mode=2))
    for _ in range(5):
        w.tick()
    check("等待前 hasBuf=TRUE", w.axis.hasBuf and fw.Busy and fw.waiting)
    w.trigger(fw, lin6(-25, 15, 300, speed=120, accel=600, buffer_mode=1))
    upgraded = False
    for i in range(3000):
        w.tick()
        if fw.running and not fw.waiting:
            upgraded = True
        if fw.InPos and not fw.Busy:
            break
    check("等待者自我升级为打断并执行",
          upgraded and fw.InPos and abs(w.axis.PlanX+25) < 2
          and abs(w.axis.PlanY-15) < 2,
          f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f})")

    # D2.5 等待者同实例改发拒绝模式：槽位释放可再缓存
    w = World6()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin6(40, 0, 300, speed=30, accel=150,
                                             buffer_mode=2))
    fw = w.new_fb('LIN'); w.trigger(fw, lin6(40, 25, 300, speed=70, accel=350,
                                             buffer_mode=2))
    for _ in range(5):
        w.tick()
    check("D2.5前置: fw等待中", fw.waiting and w.axis.hasBuf and fw.Busy)
    w.trigger(fw, lin6(0, 0, 300, buffer_mode=0))
    w.tick()
    check("D2.5: 新指令Rejected", fw.Rejected)
    check("D2.5: 旧等待CommandAborted留痕",
          fw.CommandAborted and not fw.Busy)
    check("D2.5: 缓存槽已释放(hasBuf=FALSE)", not w.axis.hasBuf)
    check("D2.5: 原活动指令不受影响",
          fv.Busy and w.axis.Busy and not fv.CommandAborted)
    f2 = w.new_fb('PTP'); w.trigger(f2, ptp6(0, 0, 315, speed=120, accel=600,
                                             buffer_mode=2))
    w.tick()
    check("D2.5: 槽位释放后可重新缓存",
          f2.waiting and w.axis.hasBuf and not f2.Rejected)
    ok = w.run_until_idle(6000)
    check("D2.5: 链路最终完成",
          ok and fv.InPos and f2.InPos and abs(w.axis.PlanZ-315) < 2
          and abs(w.axis.PlanX) < 2,
          f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f},{w.axis.PlanZ:.1f})")

    # D2.6 等待者同实例再次缓存（覆盖旧等待）
    w = World6()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin6(40, 0, 300, speed=30, accel=150,
                                             buffer_mode=2))
    fw = w.new_fb('LIN'); w.trigger(fw, lin6(40, 25, 300, speed=70, accel=350,
                                             buffer_mode=2))
    for _ in range(5):
        w.tick()
    w.trigger(fw, lin6(-25, -25, 310, speed=90, accel=450, buffer_mode=2))
    w.tick()
    check("D2.6: 覆盖缓存成功(仍等待)",
          fw.waiting and not fw.Rejected and not fw.CommandAborted
          and w.axis.hasBuf)
    ok = w.run_until_idle(6000)
    check("D2.6: 按新目标完成",
          ok and fw.InPos and abs(w.axis.PlanX+25) < 2
          and abs(w.axis.PlanY+25) < 2 and abs(w.axis.PlanZ-310) < 2,
          f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f},{w.axis.PlanZ:.1f})")

    section("D3. 链式缓存：连续4段（PTP/LIN/CIR/LIN）")
    w = World6()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
    fb1 = w.new_fb('PTP')
    fb2 = w.new_fb('LIN')
    fb3 = w.new_fb('CIR')
    fb4 = w.new_fb('LIN')
    p1 = ptp6(35, 0, 300, speed=120, accel=600, buffer_mode=2)
    p2 = lin6(35, 25, 300, speed=120, accel=600, buffer_mode=2)
    # 从(35,25)绕圆心(15,25)半径20 顺时针90° → (15,5)
    p3 = cir6(15, 5, 300, aux=(15, 25, 300), cir_mode=0,
              speed=120, accel=600, buffer_mode=2)
    p4 = lin6(0, 0, 300, speed=120, accel=600, buffer_mode=2)
    w.trigger(fb1, p1)
    w.trigger(fb2, p2)
    stage = 0
    chain_ok = False
    arrived = {}
    inpos_last = {fb: False for fb in (fb1, fb2, fb3, fb4)}
    chain = [(fb1, (35, 0)), (fb2, (35, 25)), (fb3, (15, 5)), (fb4, (0, 0))]
    for i in range(9000):
        if stage == 0 and w.axis.Busy and not w.axis.hasBuf:
            w.trigger(fb3, p3); stage = 1
        elif stage == 1 and w.axis.Busy and not w.axis.hasBuf:
            w.trigger(fb4, p4); stage = 2
        w.tick()
        for fb, _ in chain:
            if fb.InPos and not inpos_last[fb]:
                arrived[fb] = (w.axis.PlanX, w.axis.PlanY)
            inpos_last[fb] = fb.InPos
        if stage == 2 and not w.axis.Busy and not w.axis.hasBuf:
            chain_ok = True
            break
    end_ok = all(fb.InPos and not fb.Busy for fb, _ in chain)
    check("链式4段全部完成", chain_ok and end_ok)
    for fb, (tx, ty) in chain:
        p = arrived.get(fb)
        check(f"段终点≈({tx},{ty})",
              p is not None and abs(p[0]-tx) < 2 and abs(p[1]-ty) < 2,
              f"实际={p}")
    check("最终回到(0,0)", abs(w.axis.PlanX) < 2 and abs(w.axis.PlanY) < 2,
          f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f})")
    check("链中无异常abort", all(not fb.CommandAborted for fb, _ in chain))

    # ============================================================
    section("E. 同实例重触发")

    for kind in ('PTP', 'LIN'):
        w = World6()
        f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
        f = w.new_fb(kind)
        p1 = cmd_for6(kind, (40, 0, 300, 0, 0, 0), speed=30, accel=150,
                      buffer_mode=1)
        w.trigger(f, p1)
        ok = False
        for i in range(3000):
            if i == 150:
                p2 = cmd_for6(kind, (0, 35, 310, 0, 6, 0), speed=200,
                              accel=1000, buffer_mode=1)
                w.trigger(f, p2)
            w.tick()
            if i > 150 and f.InPos and not f.Busy:
                ok = abs(w.axis.PlanY-35) < 2 and abs(w.axis.PlanZ-310) < 2
                break
        check(f"{kind}同实例重触发后到达新目标", ok,
              f"({w.axis.PlanY:.1f},{w.axis.PlanZ:.1f})")

    w = World6()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(30, 0, 300)); w.run_until_idle()
    f = w.new_fb('CIR')
    w.trigger(f, cir6(20, 20, 300, aux=(20, 0, 300), speed=30, accel=150,
                      buffer_mode=1))
    ok = False
    for i in range(3000):
        if i == 150:
            w.trigger(f, cir6(0, 0, 300, aux=(20, -20, 300), cir_mode=1,
                              speed=200, accel=1000, buffer_mode=1))
        w.tick()
        if i > 150 and f.InPos and not f.Busy:
            ok = abs(w.axis.PlanX) < 2 and abs(w.axis.PlanY) < 2
            break
    check("CIR同实例重触发后到达新目标", ok,
          f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f})")

    # ============================================================
    section("F. 信号语义")

    # F1 InPos 锁存
    w = World6()
    f = w.new_fb('PTP'); w.trigger(f, ptp6(25, 0, 300))
    w.run_until_idle()
    for _ in range(30):
        w.tick()
    check("InPos 完成后持续锁存", f.InPos)
    w.trigger(f, ptp6(35, 0, 300, buffer_mode=0))
    w.tick()
    check("新上升沿清除 InPos", not f.InPos and f.Busy)
    w.run_until_idle()

    # F2 CommandAborted 锁存到下次上升沿
    w = World6()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin6(40, 0, 300, speed=30, accel=150,
                                             buffer_mode=1))
    fk = w.new_fb('PTP')
    for i in range(3000):
        if i == 100:
            w.trigger(fk, ptp6(0, 0, 315, buffer_mode=1))
        w.tick()
        if fk.InPos and not fk.Busy:
            break
    for _ in range(30):
        w.tick()
    check("CommandAborted 锁存", fv.CommandAborted)
    w.trigger(fv, lin6(0, 0, 300, speed=200, accel=1000, buffer_mode=0))
    w.tick()
    check("受害者新上升沿清除 CommandAborted",
          not fv.CommandAborted and fv.Busy)
    w.run_until_idle()

    # F3 Busy 含缓存等待期
    w = World6()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin6(40, 0, 300, speed=30, accel=150,
                                             buffer_mode=2))
    fw = w.new_fb('PTP')
    for _ in range(5):
        w.tick()
    w.trigger(fw, ptp6(0, 25, 300, buffer_mode=2))
    busy_wait = False
    for i in range(5000):
        w.tick()
        if fw.Busy and not fw.running:
            busy_wait = True
        if fw.InPos and not fw.Busy:
            break
    check("缓存等待期 Busy=TRUE", busy_wait)
    check("等待者完成", fw.InPos)

    # F4 Execute 长电平不重触发
    w = World6()
    fh = w.new_fb('PTP'); w.trigger(fh, ptp6(0, 0, 300)); w.run_until_idle()
    f = w.new_fb('PTP')
    p = ptp6(30, 0, 300)
    w.find(f)[2] = p
    busy_fell = False
    restarted = False
    for i in range(1500):
        if i < 100:
            w.hold_execute(f)
        was_busy = f.Busy
        w.tick()
        if was_busy and not f.Busy and f.InPos:
            busy_fell = True
        if busy_fell and f.Busy:
            restarted = True
    check("Execute长电平只触发一次",
          not restarted and f.InPos and abs(w.axis.PlanX-30) < 1,
          f"PlanX={w.axis.PlanX:.2f} InPos={f.InPos}")
    w.find(f)[2] = ptp6(38, 0, 300)
    second = False
    for i in range(1500):
        if i == 30:
            w.trigger(f, ptp6(38, 0, 300))
        w.tick()
        if i > 30 and f.InPos and not f.Busy and abs(w.axis.PlanX-38) < 1:
            second = True
            break
    check("重新上升沿可再次触发", second and f.InPos,
          f"PlanX={w.axis.PlanX:.2f}")

    # F5 非法参数拒绝
    w = World6()
    f = w.new_fb('PTP')
    w.trigger(f, ptp6(30, 0, 300, speed=0, accel=1000))
    w.tick()
    check("Speed<=0 被 Rejected", f.Rejected and not f.Busy and not w.axis.Busy)
    w.tick()
    check("非法参数 Rejected 也是脉冲", not f.Rejected)
    f2 = w.new_fb('LIN')
    w.trigger(f2, lin6(30, 0, 300, speed=100, accel=-1))
    w.tick()
    check("Accel<=0 被 Rejected", f2.Rejected)
    f3 = w.new_fb('PTP')
    w.trigger(f3, ptp6(30, 0, 300, speed=100, accel=1000, jerk=0, move_mode=1))
    w.tick()
    check("S曲线 Jerk<=0 被 Rejected", f3.Rejected)

    # ============================================================
    section("G. 失能恢复")

    w = World6()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin6(40, 0, 300, speed=30, accel=150))
    for i in range(150):
        w.tick()
    check("失能前运动中", fv.Busy and w.axis.Busy)
    for _ in range(5):
        w.tick(enable=False)
    check("失能后指令FB复位", not fv.Busy and not fv.InPos and not w.axis.Busy)
    check("失能后轴输出清零",
          all(w.axis.TarTheta[k] == 0 for k in range(6))
          and w.axis.CurX == 0 and w.axis.CurA == 0)
    w.sim.th = list(ik(pose_rad(HOME_DEG)))
    for _ in range(3):
        w.tick()
    f2 = w.new_fb('PTP')
    w.trigger(f2, ptp6(25, -20, 310, buffer_mode=0))
    ok = w.run_until_idle()
    check("重新使能后新指令正常执行",
          ok and f2.InPos and abs(w.axis.PlanX-25) < 1)
    check("失能期间遗留FB不会自行复活", not fv.running and fv.phase == 0)

    # ============================================================
    section("H. 随机模糊测试（20组随机序列）")

    rng_master = random.Random(20261012)
    FUZZ_SEEDS = [rng_master.randrange(1 << 30) for _ in range(20)]

    for gi, seed in enumerate(FUZZ_SEEDS):
        rng = random.Random(seed)
        w = World6(ff=rng.random() < 0.7, kff=rng.uniform(0.2, 0.9))
        f0 = w.new_fb('PTP'); w.trigger(f0, ptp6(0, 0, 300)); w.run_until_idle()

        N = rng.randint(8, 16)
        pool = []
        events = []
        t_now = 2
        kinds = ['PTP', 'LIN', 'CIR']

        def make_params(kind, bm, mm):
            if kind == 'CIR':
                p = cir_params6(rng, w.axis.CurX, w.axis.CurY, w.axis.CurZ,
                                w.axis.CurA, w.axis.CurB, w.axis.CurC)
                p['BufferMode'] = bm
                p['MoveMode'] = mm
                return p
            p6 = reachable6(rng)
            return cmd_for6(kind, p6,
                            speed=rng.uniform(80, 300),
                            accel=rng.uniform(500, 2000),
                            jerk=rng.uniform(5000, 20000),
                            move_mode=mm, buffer_mode=bm)

        for n in range(N):
            kind = rng.choice(kinds)
            existing = [p for p in pool if p[1] == kind and not p[0].Busy]
            if existing and rng.random() < 0.3:
                fb, _ = rng.choice(existing)
            else:
                fb = w.new_fb(kind)
                pool.append((fb, kind))
            bm = rng.choice([0, 0, 1, 1, 2])
            mm = rng.randint(0, 1)
            t_now += rng.randint(1, 400)
            events.append((t_now, fb, kind, bm, mm))

        deadline = t_now + 9000
        rejected = aborted = completed = fired_n = 0
        expected_term = 0
        consistency_bad = 0
        term = {id(fb): None for fb, _ in pool}
        for i in range(deadline):
            fired_this_cycle = []
            for cyc, fb, kind, bm, mm in events:
                if cyc == i:
                    fired_this_cycle.append((fb, fb.Busy, fb.waiting))
                    w.trigger(fb, make_params(kind, bm, mm))
                    fired_n += 1
                    term[id(fb)] = None
            w.tick()
            for fb, was_busy, was_waiting in fired_this_cycle:
                if fb.Rejected:
                    expected_term += 1
                elif not was_busy:
                    expected_term += 1
            if not w.owner_consistent():
                consistency_bad += 1
            for fb, _ in pool:
                cur = 'I' if fb.InPos else ('A' if fb.CommandAborted else None)
                if cur is not None and cur != term[id(fb)]:
                    term[id(fb)] = cur
                    if cur == 'I':
                        completed += 1
                    else:
                        aborted += 1
                if fb.Rejected:
                    rejected += 1
        idle = (not w.axis.Busy) and (not w.axis.hasBuf) and w.axis.owner == 0
        check(f"序列{gi+1:02d} seed={seed}: 无死锁轴回收", idle,
              f"Busy={w.axis.Busy} hasBuf={w.axis.hasBuf} owner={w.axis.owner}")
        check(f"序列{gi+1:02d}: owner/ActiveType 全程一致", consistency_bad == 0,
              f"bad={consistency_bad}")
        check(f"序列{gi+1:02d}: 角度跳变有界(<0.05)", w.max_jump < 0.05,
              f"jump={w.max_jump:.4f}")
        check(f"序列{gi+1:02d}: FK(Tar)≈Plan(<0.5mm & <0.01rad)",
              w.max_fkerr_pos < 0.5 and w.max_fkerr_ang < 0.01,
              f"pos={w.max_fkerr_pos:.3f}mm ang={w.max_fkerr_ang:.4f}rad")
        check(f"序列{gi+1:02d}: {fired_n}次触发终态闭环"
              f"({completed}完成/{aborted}打断/{rejected}拒绝)",
              completed + aborted + rejected == expected_term,
              f"fired={fired_n} expected={expected_term} "
              f"c={completed} a={aborted} r={rejected}")

    print("\n" + "=" * 64)
    print(f"矩阵测试结果: {PASS} 通过, {FAIL} 失败")
    print("=" * 64)
    if FAIL:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
