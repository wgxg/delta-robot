"""
Delta 机器人 - 大规模多样本矩阵测试
====================================
覆盖：
  A. 指令×打断 全交叉矩阵（PTP/LIN/CIR × PTP/LIN/CIR，梯形+S曲线）
  B. 打断时序：加速段 / 匀速段 / 减速段
  C. 拒绝矩阵 + 空闲拒绝模式正常执行
  D. 缓存组合：跨类型缓存、槽满、缓存等待者被打断、等待者自我升级、链式缓存
  E. 同实例重触发（自打断）
  F. 信号语义：InPos/CommandAborted 锁存、Rejected 单周期脉冲、
     Busy 含等待期、Execute 长电平不重触发
  G. 失能恢复：运动中核心失能→指令复位→重新使能可运行
  H. 随机模糊测试：20 组随机指令序列（死锁/跳变/FK一致性/owner一致性）
"""
import math
import random
from test_delta import (DeltaAxis, FB_DeltaCore, FB_PTP, FB_LIN, FB_CIR,
                        Sim, ptp, lin, cir, R, r, L, l, CYCLE,
                        COS1, SIN1, COS2, SIN2, COS3, SIN3)

# ============================================================
# 统计
# ============================================================
PASS = FAIL = 0
__all__ = ['World', 'reachable', 'cir_params', 'fk_xyz', 'section', 'check',
           'FB_CLASSES']


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


# ============================================================
# World：多 FB 实例调度器
# ============================================================
FB_CLASSES = {'PTP': FB_PTP, 'LIN': FB_LIN, 'CIR': FB_CIR}


def fk_xyz(th1, th2, th3):
    e = R - r
    s1 = e + L*math.cos(th1); s2 = e + L*math.cos(th2); s3 = e + L*math.cos(th3)
    sn1 = L*math.sin(th1); sn2 = L*math.sin(th2); sn3 = L*math.sin(th3)
    a1 = -2.0*(s2*COS2-s1*COS1); b1 = -2.0*(s2*SIN2-s1*SIN1)
    c1 = -2.0*(sn2-sn1); d1 = (s1*s1+sn1*sn1)-(s2*s2+sn2*sn2)
    a2 = -2.0*(s3*COS3-s1*COS1); b2 = -2.0*(s3*SIN3-s1*SIN1)
    c2 = -2.0*(sn3-sn1); d2 = (s1*s1+sn1*sn1)-(s3*s3+sn3*sn3)
    Det = a1*b2-a2*b1
    if abs(Det) < 1e-9:
        return None
    px = (b2*d1-b1*d2)/Det; qx = (b1*c2-b2*c1)/Det
    py = (a1*d2-a2*d1)/Det; qy = (a2*c1-a1*c2)/Det
    X0 = px-s1*COS1; Y0 = py-s1*SIN1
    Az = qx*qx+qy*qy+1.0; Bz = 2.0*(X0*qx+Y0*qy)-2.0*sn1
    Cz = X0*X0+Y0*Y0+sn1*sn1-l*l; Dz = Bz*Bz-4.0*Az*Cz
    if Dz < 0:
        return None
    z1 = (-Bz+math.sqrt(Dz))/(2.0*Az); z2 = (-Bz-math.sqrt(Dz))/(2.0*Az)
    z = z1 if z1 > 0 else (z2 if z2 > 0 else z1)
    return (px+qx*z, py+qy*z, z)


class World:
    def __init__(self, ff=True, kff=0.5):
        self.axis = DeltaAxis()
        self.core = FB_DeltaCore()
        self.sim = Sim(self.axis)
        self.fbs = []          # (fb, kind, params)
        self.pulse = set()     # 本周期 Execute 上升沿的 fb id()
        self.hold = set()      # 本周期 Execute 保持高电平的 fb id()
        self.max_jump = 0.0
        self.max_fkerr = 0.0
        self.t = 0
        self._prev_tar = (0.0, 0.0, 0.0)
        self._prev_plan = (0.0, 0.0, 300.0)
        self._plan_at_tar = (0.0, 0.0, 300.0)
        self.ff, self.kff = ff, kff
        self.sim.step(self.core, [])   # 先使能一拍

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
        """本周期 Execute 保持高电平（不产生新上升沿语义由FB内部保证）"""
        self.hold.add(id(fb))

    def find(self, fb):
        for rec in self.fbs:
            if rec[0] is fb:
                return rec

    def tick(self, enable=True):
        self.axis.CurTheta1, self.axis.CurTheta2, self.axis.CurTheta3 = self.sim.th
        # 约定顺序：先调指令FB（CurX/CurTheta 为上一拍核心结果，与真实PLC一致）
        for rec in self.fbs:
            fb, kind, p = rec
            if p is None:
                continue
            ex = (id(fb) in self.pulse) or (id(fb) in self.hold)
            fb(self.axis, Execute=ex, **p)
        self.pulse.clear(); self.hold.clear()
        # 最后调核心
        self.core(R, r, L, l, self.axis, CYCLE, enable,
                  self.ff if enable else False, self.kff if enable else 0.0)
        tar = (self.axis.TarTheta1, self.axis.TarTheta2, self.axis.TarTheta3)
        for k in range(3):
            self.sim.th[k] += (tar[k]-self.sim.th[k])*0.3
        jump = max(abs(tar[k]-self._prev_tar[k]) for k in range(3))
        self.max_jump = max(self.max_jump, jump)
        plan_moved = math.sqrt((self.axis.PlanX-self._prev_plan[0])**2 +
                               (self.axis.PlanY-self._prev_plan[1])**2 +
                               (self.axis.PlanZ-self._prev_plan[2])**2)
        tar_held = all(abs(tar[k]-self._prev_tar[k]) < 1e-12 for k in range(3))
        if not tar_held:
            self._plan_at_tar = (self.axis.PlanX, self.axis.PlanY, self.axis.PlanZ)
        # IK 失败安全保持时 Tar 不动而 Plan 继续：累计漂移 >0.5mm 后不参与一致性统计
        hold_drift = math.sqrt(
            (self.axis.PlanX-self._plan_at_tar[0])**2 +
            (self.axis.PlanY-self._plan_at_tar[1])**2 +
            (self.axis.PlanZ-self._plan_at_tar[2])**2)
        if self.axis.Busy and not (tar_held and hold_drift > 0.5):
            fkp = fk_xyz(*tar)
            if fkp is not None:
                err = math.sqrt((fkp[0]-self.axis.PlanX)**2 +
                                (fkp[1]-self.axis.PlanY)**2 +
                                (fkp[2]-self.axis.PlanZ)**2)
                self.max_fkerr = max(self.max_fkerr, err)
        self._prev_tar = tar
        self._prev_plan = (self.axis.PlanX, self.axis.PlanY, self.axis.PlanZ)
        self.t += 1

    def run_until_idle(self, timeout=6000):
        """跑到轴空闲（Busy=FALSE 且无缓存等待）"""
        for _ in range(timeout):
            self.tick()
            if not self.axis.Busy and not self.axis.hasBuf:
                return True
        return False

    def owner_consistent(self):
        """Busy 时 owner 必须等于某个 running FB 的 myId，空闲时 owner=0"""
        if not self.axis.Busy:
            return self.axis.owner == 0 and self.axis.ActiveType == 0
        for rec in self.fbs:
            fb = rec[0]
            if getattr(fb, 'running', False) and fb.myId == self.axis.owner:
                return rec[1] == ('PTP', 'LIN', 'CIR')[self.axis.ActiveType-1]
        return False


# ============================================================
# 可达目标点
# ============================================================
def reachable(rng, z_lo=230.0, z_hi=320.0, rho_hi=80.0):
    rho = rng.uniform(0, rho_hi)
    a = rng.uniform(0, 2*math.pi)
    return rho*math.cos(a), rho*math.sin(a), rng.uniform(z_lo, z_hi)


def cir_params(rng, sx, sy, sz):
    """以起点为圆弧起点生成可达的圆心式/三点式参数（圆心偏向内侧，弧不外扩）"""
    base = math.hypot(sx, sy)
    inward = math.atan2(sy, sx) + math.pi if base > 1e-6 else rng.uniform(0, 2*math.pi)
    ang = inward + rng.uniform(-math.pi/2, math.pi/2)   # 朝原点一侧半圆
    rad = rng.uniform(18, 32)
    span = rng.choice([60, 90, 120, -60, -90, -120]) * math.pi/180
    cx, cy = sx+rad*math.cos(ang), sy+rad*math.sin(ang)
    # 终点 = 圆心 + 旋转(起点-圆心, span)
    dx, dy = sx-cx, sy-cy
    ex = cx + dx*math.cos(span) - dy*math.sin(span)
    ey = cy + dx*math.sin(span) + dy*math.cos(span)
    ez = sz
    cm_x = cx + dx*math.cos(span/2) - dy*math.sin(span/2)
    cm_y = cy + dx*math.sin(span/2) + dy*math.cos(span/2)
    mode = rng.randint(0, 1)
    if mode == 0:
        # 圆心式：span 方向由短弧规则决定，辅助=圆心
        return cir(ex, ey, ez, aux=(cx, cy, sz), cir_mode=0,
                     speed=rng.uniform(100, 250), accel=rng.uniform(600, 1500),
                     jerk=8000, move_mode=rng.randint(0, 1))
    else:
        # 三点式：辅助=弧上中间点
        return cir(ex, ey, ez, aux=(cm_x, cm_y, ez), cir_mode=1,
                   speed=rng.uniform(100, 250), accel=rng.uniform(600, 1500),
                   jerk=8000, move_mode=rng.randint(0, 1))


def cmd_for(kind, x, y, z, rng=None, **kw):
    if kind == 'PTP':
        return ptp(x, y, z, **kw)
    if kind == 'LIN':
        return lin(x, y, z, **kw)
    return cir(x, y, z, aux=kw.pop('aux', (x, y-30, z)),
               cir_mode=kw.pop('cir_mode', 0), **kw)


# ============================================================
# A. 3×3 打断交叉矩阵


def main():
    # ============================================================
    section("A. 打断交叉矩阵（被打断指令 × 打断指令，含梯形/S曲线）")

    # 固定可达场景
    SCEN = {
        'PTP': dict(target=(60, 40, 310), slow=ptp(90, -70, 290, speed=60, accel=300, buffer_mode=1)),
        'LIN': dict(target=(90, 0, 300), slow=lin(95, 0, 300, speed=50, accel=250, buffer_mode=1)),
        'CIR': dict(target=(50, 50, 300), slow=None),  # 单独构造
    }
    NEW_TARGET = {'PTP': (0, 80, 320), 'LIN': (-60, 30, 300), 'CIR': (0, 0, 300)}

    for victim in ('PTP', 'LIN', 'CIR'):
        for killer in ('PTP', 'LIN', 'CIR'):
            for mm in (0, 1):
                w = World()
                w.new_fb('PTP'); w.trigger(w.fbs[0][0], ptp(0, 0, 300))
                w.run_until_idle()
                # 受害者（慢速，保证打断时仍在运动）
                fv = w.new_fb(victim)
                if victim == 'CIR':
                    # 先到圆弧起点 (100,0,300)
                    pass
                if victim == 'CIR':
                    vp = cir(50, 50, 300, aux=(50, 0, 300), cir_mode=0,
                             speed=40, accel=200, move_mode=mm, buffer_mode=1)
                else:
                    t0 = SCEN[victim]['target']
                    vp = cmd_for(victim, t0[0], t0[1], t0[2],
                                 speed=45, accel=220, move_mode=mm, buffer_mode=1)
                w.trigger(fv, vp)
                # 打断者
                fk = w.new_fb(killer)
                nt = NEW_TARGET[killer]
                if killer == 'CIR':
                    kp = cir(0, 0, 300, aux=(-30, 30, 300), cir_mode=1,
                             speed=200, accel=1000, move_mode=mm, buffer_mode=1)
                else:
                    kp = cmd_for(killer, nt[0], nt[1], nt[2],
                                 speed=200, accel=1000, move_mode=mm,
                                 buffer_mode=1)
                aborted = False
                done = False
                for i in range(2500):
                    if i == 120:
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
    # B. 打断时序：加速 / 匀速 / 减速 段
    # ============================================================
    section("B. 打断时序（慢LIN：加速段/匀速段/减速段）")

    # 慢LIN: speed=50 accel=250 dist=100 → 加速200周期(5mm), 匀速1800周期, 减速200周期
    for phase_name, hit in (("加速段", 100), ("匀速段", 1000), ("减速段", 2100)):
        w = World()
        f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
        fv = w.new_fb('LIN')
        w.trigger(fv, lin(100, 0, 300, speed=50, accel=250, buffer_mode=1))
        fk = w.new_fb('PTP')
        ok_abort = ok_done = False
        for i in range(3500):
            if i == hit:
                w.trigger(fk, ptp(0, 0, 320, speed=200, accel=1000, buffer_mode=1))
            w.tick()
            if i >= hit+1 and fv.CommandAborted:
                ok_abort = True
            if i > hit and fk.InPos and not fk.Busy:
                ok_done = True
                break
        check(f"{phase_name}打断: 受害者abort", ok_abort)
        check(f"{phase_name}打断: 新指令到达Z≈320", ok_done and abs(w.axis.PlanZ-320) < 1.5,
              f"Z={w.axis.PlanZ:.1f}")
        check(f"{phase_name}打断: 角度跳变有界", w.max_jump < 0.05, f"jump={w.max_jump:.4f}")


    # ============================================================
    # C. 拒绝矩阵
    # ============================================================
    section("C. 拒绝模式（运动中来任何指令都拒绝，原指令不受影响）")

    for victim in ('PTP', 'LIN', 'CIR'):
        w = World()
        f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
        fv = w.new_fb(victim)
        if victim == 'CIR':
            vp = cir(50, 50, 300, aux=(50, 0, 300), speed=40, accel=200, buffer_mode=0)
        else:
            t = (90, 0, 300)
            vp = cmd_for(victim, *t, speed=45, accel=220, buffer_mode=0)
        w.trigger(fv, vp)
        rej_results = {}
        for intr in ('PTP', 'LIN', 'CIR'):
            fi = w.new_fb(intr)
            if intr == 'CIR':
                ip = cir(0, 0, 300, aux=(0, -30, 300), cir_mode=1, buffer_mode=0)
            else:
                ip = cmd_for(intr, -50, 50, 320, buffer_mode=0)
            for i in range(1, 250):
                if i == 60:
                    w.trigger(fi, ip)
                w.tick()
                if i == 60:
                    rej_results[intr] = fi.Rejected
                if i == 61:
                    rej_results[intr+'_pulse'] = not fi.Rejected  # 次日自动清零
        for intr in ('PTP', 'LIN', 'CIR'):
            check(f"{victim}运动中拒绝{intr}", rej_results.get(intr, False))
            check(f"{intr} Rejected为单周期脉冲", rej_results.get(intr+'_pulse', False))
        check(f"{victim}未受影响仍在运动", fv.Busy)
        check("ActiveType 与受害者一致",
              w.axis.ActiveType == {'PTP': 1, 'LIN': 2, 'CIR': 3}[victim])

    # 空闲时 BufferMode=0 的指令也应正常启动
    w = World()
    f = w.new_fb('PTP')
    w.trigger(f, ptp(40, -30, 310, buffer_mode=0))
    idle_ok = w.run_until_idle()
    check("空闲时拒绝模式指令正常执行", idle_ok and f.InPos and abs(w.axis.PlanX-40) < 1)


    # ============================================================
    # D. 缓存组合
    # ============================================================
    section("D1. 跨类型缓存（受害者×缓存等待者）")

    for victim in ('PTP', 'LIN', 'CIR'):
        for waiter in ('PTP', 'LIN', 'CIR'):
            w = World()
            f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
            fv = w.new_fb(victim)
            if victim == 'CIR':
                vp = cir(50, 50, 300, aux=(50, 0, 300), speed=120, accel=600, buffer_mode=2)
            else:
                vp = cmd_for(victim, 90, 0, 300, speed=90, accel=450, buffer_mode=2)
            w.trigger(fv, vp)
            fw = w.new_fb(waiter)
            if waiter == 'CIR':
                # 按受害者终点程序化构造90°圆弧（圆心=起点正Y方向40mm，+90°）
                sx, sy, sz = {'PTP': (90, 0, 300), 'LIN': (90, 0, 300),
                              'CIR': (50, 50, 300)}[victim]
                ccx, ccy, ccz = sx, sy-40.0, sz
                # Rot(+90°): (dx,dy)->(-dy,dx)，S-C=(0,40) → E=C+(-40,0)
                ex, ey = ccx-40.0, ccy
                wp = cir(ex, ey, ccz, aux=(ccx, ccy, ccz), cir_mode=0,
                         speed=120, accel=600, buffer_mode=2)
                target_xy = (ex, ey)
            else:
                t = (-70, 50, 310)
                wp = cmd_for(waiter, *t, speed=120, accel=600, buffer_mode=2)
                target_xy = (-70, 50)
            waiting_seen = False
            for i in range(30):
                if i == 5:
                    w.trigger(fw, wp)
                w.tick()
                if i >= 6 and fw.Busy and not fw.running:
                    waiting_seen = True
            ok = w.run_until_idle(5000)
            tag = f"{victim}→缓存{waiter}"
            check(f"{tag}: 等待期Busy=TRUE", waiting_seen)
            check(f"{tag}: 等待者最终完成", ok and fw.InPos and not fw.Busy,
                  f"InPos={fw.InPos} Busy={fw.Busy}")
            check(f"{tag}: 到达等待者目标",
                  abs(w.axis.PlanX-target_xy[0]) < 2 and abs(w.axis.PlanY-target_xy[1]) < 2,
                  f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f})")
            check(f"{tag}: 受害者不报告abort", not fv.CommandAborted)

    section("D2. 缓存等待期间：再来指令（拒绝/槽满/打断等待者）")

    # D2.1 等待期来拒绝模式 → Rejected，等待链不受影响
    w = World()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin(90, 0, 300, speed=60, accel=300, buffer_mode=2))
    fw = w.new_fb('LIN'); w.trigger(fw, lin(90, 60, 300, speed=100, accel=500, buffer_mode=2))
    w.tick(); w.tick(); w.tick(); w.tick(); w.tick()
    fx = w.new_fb('PTP')
    rej = False
    for i in range(3000):
        if i == 0:
            w.trigger(fx, ptp(0, 0, 350, buffer_mode=0))
        w.tick()
        if i == 0:
            rej = fx.Rejected
        if fw.InPos and not fw.Busy:
            break
    check("等待期拒绝模式被Rejected", rej)
    check("等待链仍正常完成", fw.InPos and abs(w.axis.PlanY-60) < 2)
    check("插队者未执行", not fx.InPos and not fx.running and abs(w.axis.PlanZ-350) > 5)

    # D2.2 等待期来缓存模式 → 槽满 Rejected
    w = World()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin(90, 0, 300, speed=60, accel=300, buffer_mode=2))
    fw = w.new_fb('LIN'); w.trigger(fw, lin(90, 60, 300, speed=100, accel=500, buffer_mode=2))
    for _ in range(5): w.tick()
    fx = w.new_fb('CIR')
    xp = cir(0, 0, 300, aux=(30, 30, 300), cir_mode=1, buffer_mode=2)
    w.trigger(fx, xp)
    w.tick()
    check("等待期第二条缓存槽满Rejected", fx.Rejected)

    # D2.3 等待期来打断 → 等待者 CommandAborted，新指令接管，hasBuf 清空
    w = World()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin(90, 0, 300, speed=60, accel=300, buffer_mode=2))
    fw = w.new_fb('LIN'); w.trigger(fw, lin(90, 60, 300, speed=100, accel=500, buffer_mode=2))
    for _ in range(5): w.tick()
    fx = w.new_fb('PTP'); w.trigger(fx, ptp(-50, -50, 320, speed=200, accel=1000, buffer_mode=1))
    w_abort = False
    for i in range(2500):
        w.tick()
        if fw.CommandAborted:
            w_abort = True
        if fx.InPos and not fx.Busy:
            break
    check("等待者被打断时CommandAborted", w_abort)
    check("打断后hasBuf清空", not w.axis.hasBuf)
    check("插队指令完成到达(-50,-50,320)", fx.InPos and abs(w.axis.PlanX+50) < 2
          and abs(w.axis.PlanY+50) < 2 and abs(w.axis.PlanZ-320) < 2,
          f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f},{w.axis.PlanZ:.1f})")

    # D2.4 等待者同实例改发打断 → 放弃等待立即执行
    w = World()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin(90, 0, 300, speed=60, accel=300, buffer_mode=2))
    fw = w.new_fb('LIN'); w.trigger(fw, lin(90, 60, 300, speed=100, accel=500, buffer_mode=2))
    for _ in range(5): w.tick()
    check("等待前 hasBuf=TRUE", w.axis.hasBuf and fw.Busy and fw.waiting)
    # 同一 fw 实例再来一个打断模式上升沿
    w.trigger(fw, lin(-40, 20, 300, speed=150, accel=800, buffer_mode=1))
    upgraded = False
    for i in range(2500):
        w.tick()
        if fw.running and not fw.waiting:
            upgraded = True
        if fw.InPos and not fw.Busy:
            break
    check("等待者自我升级为打断并执行", upgraded and fw.InPos
          and abs(w.axis.PlanX+40) < 2 and abs(w.axis.PlanY-20) < 2,
          f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f})")

    # D2.5 等待者同实例改发拒绝模式：旧等待作废(Aborted留痕)+槽位释放可再缓存
    w = World()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin(90, 0, 300, speed=60, accel=300, buffer_mode=2))
    fw = w.new_fb('LIN'); w.trigger(fw, lin(90, 60, 300, speed=100, accel=500, buffer_mode=2))
    for _ in range(5): w.tick()
    check("D2.5前置: fw等待中", fw.waiting and w.axis.hasBuf and fw.Busy)
    # fw 自己再来一个拒绝模式上升沿
    w.trigger(fw, lin(0, 0, 300, buffer_mode=0))
    w.tick()
    check("D2.5: 新指令Rejected", fw.Rejected)
    check("D2.5: 旧等待CommandAborted留痕", fw.CommandAborted and not fw.Busy)
    check("D2.5: 缓存槽已释放(hasBuf=FALSE)", not w.axis.hasBuf)
    check("D2.5: 原活动指令不受影响", fv.Busy and w.axis.Busy and not fv.CommandAborted)
    # 槽位立即可被其他FB使用
    f2 = w.new_fb('PTP'); w.trigger(f2, ptp(0, 0, 320, speed=150, accel=800, buffer_mode=2))
    w.tick()
    check("D2.5: 槽位释放后可重新缓存", f2.waiting and w.axis.hasBuf and not f2.Rejected)
    ok = w.run_until_idle(5000)
    check("D2.5: 链路最终完成", ok and fv.InPos and f2.InPos
          and abs(w.axis.PlanZ-320) < 2 and abs(w.axis.PlanX) < 2,
          f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f},{w.axis.PlanZ:.1f})")

    # D2.6 等待者同实例再次缓存（覆盖旧等待，不abort不reject）
    w = World()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin(90, 0, 300, speed=60, accel=300, buffer_mode=2))
    fw = w.new_fb('LIN'); w.trigger(fw, lin(90, 60, 300, speed=100, accel=500, buffer_mode=2))
    for _ in range(5): w.tick()
    w.trigger(fw, lin(-40, -40, 310, speed=120, accel=600, buffer_mode=2))
    w.tick()
    check("D2.6: 覆盖缓存成功(仍等待)", fw.waiting and not fw.Rejected
          and not fw.CommandAborted and w.axis.hasBuf)
    ok = w.run_until_idle(5000)
    check("D2.6: 按新目标完成", ok and fw.InPos and abs(w.axis.PlanX+40) < 2
          and abs(w.axis.PlanY+40) < 2 and abs(w.axis.PlanZ-310) < 2,
          f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f},{w.axis.PlanZ:.1f})")

    section("D3. 链式缓存：连续4段（PTP/LIN/CIR/LIN，状态驱动确定性注入）")
    w = World()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
    fb1 = w.new_fb('PTP')
    fb2 = w.new_fb('LIN')
    fb3 = w.new_fb('CIR')
    fb4 = w.new_fb('LIN')
    p1 = ptp(80, 0, 300, speed=150, accel=800, buffer_mode=2)
    p2 = lin(80, 60, 300, speed=150, accel=800, buffer_mode=2)
    # fb3: 从(80,60)绕圆心(50,60)顺时针90° → (50,30)
    p3 = cir(50, 30, 300, aux=(50, 60, 300), cir_mode=0,
             speed=150, accel=800, buffer_mode=2)
    p4 = lin(0, 0, 300, speed=150, accel=800, buffer_mode=2)
    chain = [(fb1, (80, 0)), (fb2, (80, 60)), (fb3, (50, 30)), (fb4, (0, 0))]
    w.trigger(fb1, p1)
    w.trigger(fb2, p2)
    stage = 0
    chain_ok = False
    arrived = {}     # fb -> 完成时刻的 Plan 位置
    inpos_last = {fb: False for fb, _ in chain}
    for i in range(8000):
        # 槽位一空（后一条已接管）就立刻挂上下一条
        if stage == 0 and w.axis.Busy and not w.axis.hasBuf:
            w.trigger(fb3, p3); stage = 1
        elif stage == 1 and w.axis.Busy and not w.axis.hasBuf:
            w.trigger(fb4, p4); stage = 2
        w.tick()
        for fb, _ in chain:
            if fb.InPos and not inpos_last[fb]:
                arrived[fb] = (w.axis.PlanX, w.axis.PlanY, w.axis.PlanZ)
            inpos_last[fb] = fb.InPos
        if stage == 2 and not w.axis.Busy and not w.axis.hasBuf:
            chain_ok = True
            break
    end_ok = all(fb.InPos and not fb.Busy for fb, _ in chain)
    check("链式4段全部完成", chain_ok and end_ok)
    for fb, (tx, ty) in chain:
        p = arrived.get(fb)
        check(f"段终点≈({tx},{ty})", p is not None and abs(p[0]-tx) < 2 and abs(p[1]-ty) < 2,
              f"实际={p}")
    check("最终到达(0,0)", abs(w.axis.PlanX) < 2 and abs(w.axis.PlanY) < 2,
          f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f})")
    check("链中无异常abort", all(not fb.CommandAborted for fb, _ in chain))


    # ============================================================
    # E. 同实例重触发（自打断）
    # ============================================================
    section("E. 同实例重触发")

    for kind in ('PTP', 'LIN'):
        w = World()
        f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
        f = w.new_fb(kind)
        p1 = cmd_for(kind, 95, 0, 300, speed=50, accel=250, buffer_mode=1)
        w.trigger(f, p1)
        retriggered = False
        for i in range(2000):
            if i == 150:
                p2 = cmd_for(kind, 0, 80, 310, speed=200, accel=1000, buffer_mode=1)
                w.trigger(f, p2)
            w.tick()
            if i > 150 and f.InPos and not f.Busy:
                retriggered = True
                break
        check(f"{kind}同实例重触发后到达新目标", retriggered
              and abs(w.axis.PlanY-80) < 2 and abs(w.axis.PlanZ-310) < 2,
              f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f},{w.axis.PlanZ:.1f})")

    # CIR 同实例重触发
    w = World()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp(100, 0, 300)); w.run_until_idle()
    f = w.new_fb('CIR')
    w.trigger(f, cir(50, 50, 300, aux=(50, 0, 300), speed=40, accel=200, buffer_mode=1))
    ok = False
    for i in range(2000):
        if i == 150:
            w.trigger(f, cir(0, 0, 300, aux=(50, -50, 300), cir_mode=1,
                             speed=200, accel=1000, buffer_mode=1))
        w.tick()
        if i > 150 and f.InPos and not f.Busy:
            ok = abs(w.axis.PlanX) < 2 and abs(w.axis.PlanY) < 2
            break
    check("CIR同实例重触发后到达新目标", ok, f"({w.axis.PlanX:.1f},{w.axis.PlanY:.1f})")


    # ============================================================
    # F. 信号语义
    # ============================================================
    section("F. 信号语义")

    # F1 InPos 锁存
    w = World()
    f = w.new_fb('PTP'); w.trigger(f, ptp(40, 0, 300))
    w.run_until_idle()
    for _ in range(30):
        w.tick()
    check("InPos 完成后持续锁存", f.InPos)
    w.trigger(f, ptp(60, 0, 300, buffer_mode=0))
    w.tick()
    check("新上升沿清除 InPos", not f.InPos and f.Busy)
    w.run_until_idle()

    # F2 CommandAborted 锁存到下次上升沿
    w = World()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin(90, 0, 300, speed=50, accel=250, buffer_mode=1))
    fk = w.new_fb('PTP')
    for i in range(2000):
        if i == 100:
            w.trigger(fk, ptp(0, 0, 320, buffer_mode=1))
        w.tick()
        if fk.InPos and not fk.Busy:
            break
    for _ in range(30):
        w.tick()
    check("CommandAborted 锁存", fv.CommandAborted)
    # 受害者重新接单（被拒绝不会清；新上升沿清除）
    w.trigger(fv, lin(0, 0, 300, speed=200, accel=1000, buffer_mode=0))
    w.tick()
    check("受害者新上升沿清除 CommandAborted", not fv.CommandAborted and fv.Busy)
    w.run_until_idle()

    # F3 Busy 包含缓存等待期
    w = World()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin(90, 0, 300, speed=60, accel=300, buffer_mode=2))
    fw = w.new_fb('PTP')
    w.tick(); w.tick(); w.tick(); w.tick(); w.tick()
    w.trigger(fw, ptp(0, 60, 300, buffer_mode=2))
    busy_wait = False
    for i in range(4000):
        w.tick()
        if fw.Busy and not fw.running:
            busy_wait = True
        if fw.InPos and not fw.Busy:
            break
    check("缓存等待期 Busy=TRUE", busy_wait)
    check("等待者完成", fw.InPos)

    # F4 Execute 长电平不重触发（连续100周期保持高电平，只能启动一次）
    w = World()
    fh = w.new_fb('PTP'); w.trigger(fh, ptp(0, 0, 300)); w.run_until_idle()
    f = w.new_fb('PTP')
    p = ptp(50, 0, 300)
    w.find(f)[2] = p
    busy_fell = False
    restarted = False
    for i in range(1200):
        if i < 100:
            w.hold_execute(f)
        was_busy = f.Busy
        w.tick()
        if was_busy and not f.Busy and f.InPos:
            busy_fell = True
        if busy_fell and f.Busy:
            restarted = True   # 完成后又因高电平重启
    check("Execute长电平只触发一次", not restarted and f.InPos and abs(w.axis.PlanX-50) < 1,
          f"PlanX={w.axis.PlanX:.2f} InPos={f.InPos}")
    # 低电平若干拍后再来上升沿 → 可触发第二次（边沿语义验证）
    w.find(f)[2] = ptp(80, 0, 300)
    second = False
    for i in range(1200):
        if i == 30:
            w.trigger(f, ptp(80, 0, 300))
        w.tick()
        if i > 30 and f.InPos and not f.Busy and abs(w.axis.PlanX-80) < 1:
            second = True
            break
    check("重新上升沿可再次触发", second and f.InPos,
          f"PlanX={w.axis.PlanX:.2f}")

    # F5 非法参数拒绝（Speed<=0）
    w = World()
    f = w.new_fb('PTP')
    w.trigger(f, ptp(50, 0, 300, speed=0, accel=1000))
    w.tick()
    check("Speed<=0 被 Rejected", f.Rejected and not f.Busy and not w.axis.Busy)
    w.tick()
    check("非法参数 Rejected 也是脉冲", not f.Rejected)
    f2 = w.new_fb('LIN')
    w.trigger(f2, lin(50, 0, 300, speed=100, accel=-1))
    w.tick()
    check("Accel<=0 被 Rejected", f2.Rejected)
    f3 = w.new_fb('PTP')
    w.trigger(f3, ptp(50, 0, 300, speed=100, accel=1000, jerk=0, move_mode=1))
    w.tick()
    check("S曲线 Jerk<=0 被 Rejected", f3.Rejected)


    # ============================================================
    # G. 失能 / 恢复
    # ============================================================
    section("G. 失能恢复")

    w = World()
    f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()
    fv = w.new_fb('LIN'); w.trigger(fv, lin(90, 0, 300, speed=50, accel=250))
    for i in range(150):
        w.tick()
    check("失能前运动中", fv.Busy and w.axis.Busy)
    for _ in range(5):
        w.tick(enable=False)
    check("失能后指令FB复位", not fv.Busy and not fv.InPos and not w.axis.Busy)
    check("失能后轴输出清零", w.axis.TarTheta1 == 0 and w.axis.CurX == 0)
    # 模拟实际电机/反馈也回到零位
    w.sim.th = [0.0, 0.0, 0.0]
    for _ in range(3):
        w.tick()   # 重新使能
    f2 = w.new_fb('PTP')
    w.trigger(f2, ptp(40, -30, 310, buffer_mode=0))
    ok = w.run_until_idle()
    check("重新使能后新指令正常执行", ok and f2.InPos and abs(w.axis.PlanX-40) < 1)
    check("失能期间遗留FB不会自行复活", not fv.running and fv.phase == 0)


    # ============================================================
    # H. 随机模糊测试
    # ============================================================
    section("H. 随机模糊测试（20组随机序列）")

    rng_master = random.Random(20261011)
    FUZZ_SEEDS = [rng_master.randrange(1 << 30) for _ in range(20)]

    for gi, seed in enumerate(FUZZ_SEEDS):
        rng = random.Random(seed)
        w = World(ff=rng.random() < 0.7, kff=rng.uniform(0.2, 0.9))
        # 先回零
        f0 = w.new_fb('PTP'); w.trigger(f0, ptp(0, 0, 300)); w.run_until_idle()

        N = rng.randint(8, 16)
        pool = []     # (fb, kind)
        events = []   # (cycle, fb, gen参数的闭包)
        t_now = 2
        kinds = ['PTP', 'LIN', 'CIR']

        def make_params(kind, bm, mm):
            # 注意：圆弧参数必须在触发时刻按当时 CurX/Y/Z 生成
            if kind == 'CIR':
                p = cir_params(rng, w.axis.CurX, w.axis.CurY, w.axis.CurZ)
                p['BufferMode'] = bm
                return p
            x, y, z = reachable(rng)
            return cmd_for(kind, x, y, z,
                           speed=rng.uniform(80, 300), accel=rng.uniform(500, 2000),
                           jerk=rng.uniform(5000, 20000), move_mode=mm, buffer_mode=bm)

        for n in range(N):
            kind = rng.choice(kinds)
            # 70% 用新实例，30% 复用池中已结束的同类型实例
            existing = [p for p in pool if p[1] == kind and not p[0].Busy]
            if existing and rng.random() < 0.3:
                fb, _ = rng.choice(existing)
            else:
                fb = w.new_fb(kind)
                pool.append((fb, kind))
            bm = rng.choice([0, 0, 1, 1, 2])   # 加权
            mm = rng.randint(0, 1)
            t_now += rng.randint(1, 400)       # 严格递增：一扫描最多一个上升沿
            events.append((t_now, fb, kind, bm, mm))

        deadline = t_now + 8000
        rejected = aborted = completed = fired_n = 0
        expected_term = 0   # 按指令生命周期计算的应有终态数
        consistency_bad = 0
        # 终态边沿统计：每个被触发指令最终必须恰好落入 完成/打断/拒绝 之一
        term = {id(fb): None for fb, _ in pool}
        for i in range(deadline):
            fired_this_cycle = []
            for cyc, fb, kind, bm, mm in events:
                if cyc == i:
                    # 触发前该实例状态：Busy(waiting/running) 时新触发属于"合并"
                    fired_this_cycle.append((fb, fb.Busy, fb.waiting))
                    w.trigger(fb, make_params(kind, bm, mm))
                    fired_n += 1
                    term[id(fb)] = None   # 新上升沿清除旧终态记录
            w.tick()
            # 触发后立即按仲裁结果记账（语义模型）：
            #   Rejected       → 1 个新终态（Rejected脉冲）；若旧生命周期在等待，
            #                     其 Aborted 终态属于启动时已记账的生命周期，不重复计
            #   未拒绝且原空闲 → 新生命周期，最终必有 1 个 InPos/Aborted
            #   未拒绝且原Busy → 与同实例旧生命周期合并，终态总数不增加
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
        check(f"序列{gi+1:02d}: FK(TarTheta)≈Plan(<0.5mm)", w.max_fkerr < 0.5,
              f"err={w.max_fkerr:.3f}")
        check(f"序列{gi+1:02d}: {fired_n}次触发终态闭环({completed}完成/{aborted}打断/{rejected}拒绝)",
              completed + aborted + rejected == expected_term,
              f"fired={fired_n} expected={expected_term} c={completed} a={aborted} r={rejected}")


    # ============================================================
    # 汇总
    # ============================================================
    print("\n" + "=" * 64)
    print(f"矩阵测试结果: {PASS} 通过, {FAIL} 失败")
    print("=" * 64)
    if FAIL:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
