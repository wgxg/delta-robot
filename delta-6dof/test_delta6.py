"""
Delta 6DOF 运动控制库 - PLCopen 架构多样本测试（P1）
====================================================
与 3DOF test_delta.py 同构，差异仅在几何层：
  · DeltaAxis6   共享轴状态：6 电机 + 位姿(XYZABC，姿态对外用度) + 调度令牌
  · _CommandFB   调度 mixin —— 与 3DOF 【逐字一致】，仅新增 geom_ok 形参
                  （IK 预检/圆弧退化在进入仲裁前算出，非法即走 Speed<=0 同分支）
  · FB_PTP6/LIN6/CIR6 指令FB：PTP 关节空间 6 轴 s 插补；
                  LIN 位置 lerp + 姿态四元数 slerp；CIR 位置圆弧 + 姿态 slerp
  · FB_DeltaKinematics6：包 kinematics6.ik/fk/ik_vel（FK 温启动 + 失败保持）
  · FB_DeltaCore6 纯引擎：发布参数、空闲 1:1 跟随、Busy 时 Tar=IK(Plan)+前馈
调用顺序（每周期）：写 CurTheta1..6 → 调各指令FB → 调 FB_DeltaCore6

单位约定：轴结构 Plan/Cur 的 A/B/C 用【度】，PlanVA/B/C 用【度每秒】；
         kinematics6 内部用弧度，由运动学 FB 边界转换。
"""
import math

from kinematics6 import (PARAMS, ik, fk, fk_track, ik_vel, pose_deg, pose_rad,
                         DEFAULT_GUESS, DEFAULT_REF)

CYCLE = 0.001  # 1ms
K_ANG = 100.0  # 姿态当量系数 mm/rad：路径长度 L=hypot(平移mm, K_ANG*转角rad)
HOME_DEG = (0.0, 0.0, 300.0, 0.0, 0.0, 0.0)   # 上电零位（度位姿）


# ============================================================
# 四元数（内部姿态插补用，避免欧拉角路径失真）
# ============================================================
def _qmul(p, q):
    return (p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3],
            p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2],
            p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1],
            p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0])


def rpy_to_quat(a, b, c):
    """RPY(ZYX,弧度) → 四元数 (w,x,y,z)，R=Rz(C)Ry(B)Rx(A)"""
    ha, hb, hc = a/2, b/2, c/2
    ca, sa = math.cos(ha), math.sin(ha)
    cb, sb = math.cos(hb), math.sin(hb)
    cc, sc = math.cos(hc), math.sin(hc)
    return _qmul(_qmul((cc, 0, 0, sc), (cb, 0, sb, 0)), (ca, sa, 0, 0))


def quat_to_rpy(q):
    """四元数 → RPY(弧度)，ZYX 提取（软限制 tilt≤13°，远离万向锁）"""
    w, x, y, z = q
    a = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    sb = 2*(w*y - z*x)
    b = math.asin(max(-1.0, min(1.0, sb)))
    c = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return a, b, c


def slerp(q0, q1, t):
    """单位四元数球面线性插值（自动取短弧）"""
    d = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3]
    if d < 0:
        q1 = tuple(-v for v in q1)
        d = -d
    if d > 0.9995:
        q = tuple(q0[i] + t*(q1[i]-q0[i]) for i in range(4))
        n = math.sqrt(sum(v*v for v in q))
        return tuple(v/n for v in q)
    th = math.acos(d)
    s = math.sin(th)
    k0 = math.sin((1-t)*th)/s
    k1 = math.sin(t*th)/s
    return tuple(k0*q0[i] + k1*q1[i] for i in range(4))


def quat_angle(q0, q1):
    """两姿态间短弧转角(rad)"""
    d = abs(q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3])
    return 2.0 * math.acos(min(1.0, d))


def axis_cur_pose(axis):
    return (axis.CurX, axis.CurY, axis.CurZ,
            axis.CurA, axis.CurB, axis.CurC)


def fd_rates(pose_fn, s, s_vel, ds=1e-6):
    """中心差分求 d位姿/dt：pose_fn(s) 弧度位姿，sVel 归一化进度速度 1/s。
    返回 (vx,vy,vz mm/s, Ȧ,Ḃ,Ċ rad/s)。末段 s+ds 截断到 1。"""
    s0 = min(s, 1.0)
    s1 = min(s + ds, 1.0)
    p0 = pose_fn(s0)
    p1 = pose_fn(s1)
    return tuple((p1[i]-p0[i])/ds * s_vel if s1 > s0 else 0.0
                 for i in range(6))


# ============================================================
# st_DeltaAxis6
# ============================================================
class DeltaAxis6:
    def __init__(self):
        # 机械参数（核心写，指令读）
        self.R = self.r = self.L = self.l = 0.0
        self.Rac = self.ra = self.La = self.lb = 0.0
        self.CycleTime = CYCLE
        self.FfEnable = False
        self.Kff = 0.0
        self.Enable = False
        # 反馈输入
        self.CurTheta = [0.0] * 6
        # 核心输出
        self.TarTheta = [0.0] * 6
        self.CurX = self.CurY = self.CurZ = 0.0
        self.CurA = self.CurB = self.CurC = 0.0     # 度
        self.FkError = False
        # 调度状态
        self.Busy = False
        self.ActiveType = 0       # 0空闲 1PTP 2LIN 3CIR
        self.owner = 0
        self.cmdCount = 0
        self.InPos = False
        self.hasBuf = False
        # 规划输出
        self.PlanX = self.PlanY = self.PlanZ = 0.0
        self.PlanA = self.PlanB = self.PlanC = 0.0  # 度
        self.PlanVX = self.PlanVY = self.PlanVZ = 0.0
        self.PlanVA = self.PlanVB = self.PlanVC = 0.0  # 度/s


# ============================================================
# 指令FB公共调度逻辑（mixin）—— 与 3DOF 逐字一致，仅多 geom_ok 形参
# ============================================================
class _CommandFB:
    """上升沿仲裁 + 缓存/打断令牌协调，几何与轨迹由子类实现"""

    def _reset_outputs(self):
        self.Busy = False
        self.InPos = False
        self.CommandAborted = False
        self.Rejected = False
        self.running = False
        self.waiting = False
        self.phase = 0
        self.s = 0.0

    def _schedule(self, axis, Execute, Speed, Accel, Jerk, MoveMode, BufferMode,
                  geom_ok=True):
        """返回 wantStart。失能返回 None（调用方应直接 return）"""
        if not axis.Enable:
            self._reset_outputs()
            self.exLast = Execute
            return None

        self.Rejected = False
        exRise = Execute and not self.exLast
        self.exLast = Execute
        wantStart = False

        if exRise:
            # 若本实例正在缓存槽等待：旧待执行指令作废并释放槽位，再按新指令仲裁；
            # 新指令被拒绝/槽满时，旧指令以 CommandAborted 留痕
            was_waiting = self.waiting
            if was_waiting:
                axis.hasBuf = False
                self.waiting = False
                self.Busy = False

            self.InPos = False
            self.CommandAborted = False
            if Speed <= 0 or Accel <= 0 or (MoveMode == 1 and Jerk <= 0) or not geom_ok:
                self.Rejected = True
                if was_waiting:
                    self.CommandAborted = True
            elif self.running and axis.owner == self.myId:
                wantStart = True                      # 同实例重触发
            elif axis.Busy:
                if BufferMode == 0:
                    self.Rejected = True
                    if was_waiting:
                        self.CommandAborted = True
                elif BufferMode == 1:
                    wantStart = True                  # 打断
                elif BufferMode == 2:
                    if axis.hasBuf:
                        self.Rejected = True          # 缓存槽满
                        if was_waiting:
                            self.CommandAborted = True
                    else:
                        self.waiting = True
                        axis.hasBuf = True
                        self.Busy = True
            else:
                wantStart = True

        if self.waiting:
            if not axis.hasBuf:
                self.waiting = False
                self.Busy = False
                self.CommandAborted = True            # 缓存被打断清空
            elif not axis.Busy:
                self.waiting = False
                axis.hasBuf = False
                wantStart = True

        return wantStart

    def _take_token(self, axis, active_type):
        axis.cmdCount += 1
        self.myId = axis.cmdCount
        axis.owner = self.myId
        axis.ActiveType = active_type
        axis.Busy = True
        axis.InPos = False
        axis.hasBuf = False
        self.running = True
        self.Busy = True
        self.InPos = False

    def _check_abort(self, axis):
        if self.running and axis.owner != self.myId:
            self.running = False
            self.phase = 0
            self.Busy = False
            self.CommandAborted = True

    def _finish(self, axis):
        self.running = False
        self.Busy = False
        self.InPos = True
        axis.Busy = False
        axis.ActiveType = 0
        axis.owner = 0
        axis.InPos = True

    # 梯形/S 曲线推进（3DOF 同型，返回更新后的 s；phase=4 表示走完）
    def _profile(self, axis, move_mode):
        s_rem = 1.0 - self.s
        if move_mode == 0:
            s_stop = self.sVel**2/(2.0*self.maxSAcc)
        else:
            s_stop = self.sVel**2/(2.0*self.maxSAcc) + \
                     self.sVel*self.maxSAcc/(2.0*self.maxSJerk)

        if self.phase == 1:
            if s_rem <= s_stop:
                self.phase = 3
            else:
                if move_mode == 0:
                    self.sAcc = self.maxSAcc
                else:
                    self.sAcc = min(self.sAcc + self.maxSJerk*axis.CycleTime,
                                    self.maxSAcc)
                self.sVel += self.sAcc*axis.CycleTime
                if self.sVel >= self.maxSVel:
                    self.sVel = self.maxSVel
                    self.sAcc = 0.0
                    self.phase = 2
        elif self.phase == 2:
            if s_rem <= s_stop:
                self.phase = 3
            else:
                self.sVel = self.maxSVel
                self.sAcc = 0.0
        elif self.phase == 3:
            if move_mode == 0:
                self.sAcc = -self.maxSAcc
            else:
                self.sAcc = max(self.sAcc - self.maxSJerk*axis.CycleTime,
                                -self.maxSAcc)
            self.sVel += self.sAcc*axis.CycleTime
            if self.sVel <= 0:
                self.sVel = self.sAcc = 0.0
                self.s = 1.0
                self.phase = 4

        if self.phase != 4:
            self.s = min(self.s + self.sVel*axis.CycleTime, 1.0)

    def _emit_plan(self, axis, pose_r, rates_r):
        pd = pose_deg(pose_r)
        axis.PlanX, axis.PlanY, axis.PlanZ = pd[0], pd[1], pd[2]
        axis.PlanA, axis.PlanB, axis.PlanC = pd[3], pd[4], pd[5]
        axis.PlanVX, axis.PlanVY, axis.PlanVZ = rates_r[0], rates_r[1], rates_r[2]
        axis.PlanVA = math.degrees(rates_r[3])
        axis.PlanVB = math.degrees(rates_r[4])
        axis.PlanVC = math.degrees(rates_r[5])

    def _emit_hold(self, axis, pose_deg6):
        axis.PlanX, axis.PlanY, axis.PlanZ = pose_deg6[0:3]
        axis.PlanA, axis.PlanB, axis.PlanC = pose_deg6[3:6]
        axis.PlanVX = axis.PlanVY = axis.PlanVZ = 0.0
        axis.PlanVA = axis.PlanVB = axis.PlanVC = 0.0


# ============================================================
# FB_PTP6（关节空间 6 轴同步 s 插补）
# ============================================================
class FB_PTP6(_CommandFB):
    ACTIVE_TYPE = 1

    def __init__(self):
        self.Busy = self.InPos = self.CommandAborted = self.Rejected = False
        self.running = self.waiting = False
        self.phase = 0
        self.s = self.sVel = self.sAcc = 0.0
        self.maxSVel = self.maxSAcc = self.maxSJerk = 0.0
        self.myId = 0
        self.exLast = False
        self.startTh = [0.0] * 6
        self.tgtTh = [0.0] * 6
        self.tgtPoseDeg = (0.0, 0.0, 300.0, 0.0, 0.0, 0.0)
        self._fkg = pose_rad(HOME_DEG)

    def _plan_pose(self, s):
        ang = [self.startTh[k] + (self.tgtTh[k]-self.startTh[k])*s
               for k in range(6)]
        # LM 跟踪：关节插值沿程必须保持与起终点同一装配（纯牛顿会滑根）
        res = fk_track(ang, guess=self._fkg)
        if res is None:
            res = fk(ang, guess=self._fkg)
        if res is None:
            return None
        self._fkg = res[0]
        return res[0]

    def __call__(self, axis, Execute, PositionX, PositionY, PositionZ,
                 PositionA=0.0, PositionB=0.0, PositionC=0.0,
                 Speed=200.0, Accel=1000.0, Jerk=10000.0,
                 MoveMode=0, BufferMode=0):
        tgt_deg = (PositionX, PositionY, PositionZ,
                   PositionA, PositionB, PositionC)
        rise = Execute and not self.exLast
        geom_ok = True
        if rise:
            cur = list(axis.CurTheta)
            tgt = ik(pose_rad(tgt_deg), theta_ref=tuple(cur))
            geom_ok = tgt is not None
            if geom_ok:
                self.tgtTh = list(tgt)
                self.startTh = cur
                self.tgtPoseDeg = tgt_deg

        wantStart = self._schedule(axis, Execute, Speed, Accel, Jerk,
                                   MoveMode, BufferMode, geom_ok)
        if wantStart is None:
            return

        if wantStart:
            self._take_token(axis, self.ACTIVE_TYPE)
            # 缓存提升在后续周期发生：起终点关节角按实际提升时刻 Cur 重算
            cur = list(axis.CurTheta)
            tgt = ik(pose_rad(self.tgtPoseDeg), theta_ref=tuple(cur))
            if tgt is not None:
                self.startTh = cur
                self.tgtTh = list(tgt)
            cur_r = pose_rad(axis_cur_pose(axis))
            tgt_r = pose_rad(self.tgtPoseDeg)
            dpos = math.sqrt(sum((tgt_r[i]-cur_r[i])**2 for i in range(3)))
            drot = quat_angle(rpy_to_quat(*cur_r[3:6]),
                              rpy_to_quat(*tgt_r[3:6]))
            total = math.sqrt(dpos*dpos + (K_ANG*drot)**2)
            self._fkg = cur_r
            if tgt is None or total < 1e-4:
                self._emit_hold(axis, self.tgtPoseDeg)
                self.s, self.phase = 1.0, 4
            else:
                self.maxSVel = Speed/total
                self.maxSAcc = Accel/total
                self.maxSJerk = Jerk/total
                self.s = self.sVel = self.sAcc = 0.0
                self.phase = 1

        self._check_abort(axis)

        if self.running and self.phase in (1, 2, 3):
            self._profile(axis, MoveMode)
            if self.phase != 4:
                pr = self._plan_pose(self.s)
                if pr is not None:
                    rates = fd_rates(self._plan_pose, self.s, self.sVel)
                    self._emit_plan(axis, pr, rates)
            if self.phase == 4:
                self._emit_hold(axis, self.tgtPoseDeg)

        if self.running and self.phase == 4:
            self._finish(axis)


# ============================================================
# FB_LIN6（位置线性 + 姿态 slerp）
# ============================================================
class FB_LIN6(_CommandFB):
    ACTIVE_TYPE = 2

    def __init__(self):
        self.Busy = self.InPos = self.CommandAborted = self.Rejected = False
        self.running = self.waiting = False
        self.phase = 0
        self.s = self.sVel = self.sAcc = 0.0
        self.maxSVel = self.maxSAcc = self.maxSJerk = 0.0
        self.myId = 0
        self.exLast = False
        self.startDeg = HOME_DEG
        self.tgtDeg = HOME_DEG

    def _plan_pose(self, s):
        t = tuple(self.startDeg[i] +
                  (self.tgtDeg[i]-self.startDeg[i])*s for i in range(3))
        q = slerp(self.q0, self.q1, s)
        a, b, c = quat_to_rpy(q)
        return (t[0], t[1], t[2], a, b, c)

    def __call__(self, axis, Execute, PositionX, PositionY, PositionZ,
                 PositionA=0.0, PositionB=0.0, PositionC=0.0,
                 Speed=200.0, Accel=1000.0, Jerk=10000.0,
                 MoveMode=0, BufferMode=0):
        tgt_deg = (PositionX, PositionY, PositionZ,
                   PositionA, PositionB, PositionC)
        rise = Execute and not self.exLast
        geom_ok = True
        if rise:
            geom_ok = ik(pose_rad(tgt_deg),
                         theta_ref=tuple(axis.CurTheta)) is not None
            if geom_ok:
                self.tgtDeg = tgt_deg
                self.startDeg = axis_cur_pose(axis)

        wantStart = self._schedule(axis, Execute, Speed, Accel, Jerk,
                                   MoveMode, BufferMode, geom_ok)
        if wantStart is None:
            return

        if wantStart:
            self._take_token(axis, self.ACTIVE_TYPE)
            # 缓存提升在后续周期发生：起点位姿按实际提升时刻 Cur 重取
            self.startDeg = axis_cur_pose(axis)
            sr = pose_rad(self.startDeg)
            tr = pose_rad(self.tgtDeg)
            self.q0 = rpy_to_quat(*sr[3:6])
            self.q1 = rpy_to_quat(*tr[3:6])
            dv = [tr[i]-sr[i] for i in range(3)]
            dpos = math.sqrt(sum(v*v for v in dv))
            drot = quat_angle(self.q0, self.q1)
            total = math.sqrt(dpos*dpos + (K_ANG*drot)**2)
            if total < 1e-4:
                self._emit_hold(axis, self.tgtDeg)
                self.s, self.phase = 1.0, 4
            else:
                self.maxSVel = Speed/total
                self.maxSAcc = Accel/total
                self.maxSJerk = Jerk/total
                self.s = self.sVel = self.sAcc = 0.0
                self.phase = 1

        self._check_abort(axis)

        if self.running and self.phase in (1, 2, 3):
            self._profile(axis, MoveMode)
            if self.phase != 4:
                pr = self._plan_pose(self.s)
                rates = fd_rates(self._plan_pose, self.s, self.sVel)
                self._emit_plan(axis, pr, rates)
            if self.phase == 4:
                self._emit_hold(axis, self.tgtDeg)

        if self.running and self.phase == 4:
            self._finish(axis)


# ============================================================
# FB_CIR6（位置圆弧 + 姿态沿弧 slerp）
# ============================================================
class FB_CIR6(_CommandFB):
    ACTIVE_TYPE = 3

    def __init__(self):
        self.Busy = self.InPos = self.CommandAborted = self.Rejected = False
        self.running = self.waiting = False
        self.phase = 0
        self.s = self.sVel = self.sAcc = 0.0
        self.maxSVel = self.maxSAcc = self.maxSJerk = 0.0
        self.myId = 0
        self.exLast = False
        self.startDeg = HOME_DEG
        self.tgtDeg = HOME_DEG
        self._aux = (0.0, 0.0, 0.0)
        self._cirMode = 0

    def _build_geom(self, axis, tgt_deg, aux, cir_mode):
        """返回 True/False；成功时缓存圆弧几何（与 3DOF 同公式）"""
        sx, sy, sz = axis.CurX, axis.CurY, axis.CurZ
        ex, ey, ez = tgt_deg[0], tgt_deg[1], tgt_deg[2]
        geom_fail = False
        if cir_mode == 0:
            cx, cy, cz = aux
        else:
            mx, my, mz = aux
            sm = [mx-sx, my-sy, mz-sz]
            se = [ex-sx, ey-sy, ez-sz]
            n = [sm[1]*se[2]-sm[2]*se[1], sm[2]*se[0]-sm[0]*se[2],
                 sm[0]*se[1]-sm[1]*se[0]]
            n_len = math.sqrt(sum(v*v for v in n))
            if n_len < 1e-6:
                return False
            n = [v/n_len for v in n]
            sm_len = math.sqrt(sum(v*v for v in sm))
            u = [v/sm_len for v in sm]
            v = [n[1]*u[2]-n[2]*u[1], n[2]*u[0]-n[0]*u[2],
                 n[0]*u[1]-n[1]*u[0]]
            eu = sum(se[k]*u[k] for k in range(3))
            ev = sum(se[k]*v[k] for k in range(3))
            aa = sm_len/2.0
            if abs(ev) < 1e-6:
                return False
            bb = (eu*eu+ev*ev-2.0*aa*eu)/(2.0*ev)
            cx = sx+aa*u[0]+bb*v[0]
            cy = sy+aa*u[1]+bb*v[1]
            cz = sz+aa*u[2]+bb*v[2]

        cs = [sx-cx, sy-cy, sz-cz]
        radius = math.sqrt(sum(v*v for v in cs))
        ce = [ex-cx, ey-cy, ez-cz]
        if radius < 1e-6:
            return False
        u = [v/radius for v in cs]
        if cir_mode == 0:
            n = [cs[1]*ce[2]-cs[2]*ce[1], cs[2]*ce[0]-cs[0]*ce[2],
                 cs[0]*ce[1]-cs[1]*ce[0]]
            n_len = math.sqrt(sum(v*v for v in n))
            if n_len < 1e-6:
                return False
            n = [v/n_len for v in n]
        v = [n[1]*u[2]-n[2]*u[1], n[2]*u[0]-n[0]*u[2],
             n[0]*u[1]-n[1]*u[0]]
        eu = sum(ce[k]*u[k] for k in range(3))
        ev = sum(ce[k]*v[k] for k in range(3))
        theta_end = math.atan2(ev, eu)
        if cir_mode == 1:
            cm = [aux[0]-cx, aux[1]-cy, aux[2]-cz]
            theta_mid = math.atan2(sum(cm[k]*v[k] for k in range(3)),
                                   sum(cm[k]*u[k] for k in range(3)))
            if theta_end < 0:
                theta_end += 2*math.pi
            if theta_mid < 0:
                theta_mid += 2*math.pi
            if not (theta_mid > 0 and theta_mid < theta_end):
                theta_end -= 2*math.pi
        else:
            if theta_end < 0:
                theta_end += 2*math.pi
            if theta_end > math.pi:
                theta_end -= 2*math.pi
        arc_len = radius*abs(theta_end)
        self.cx, self.cy, self.cz = cx, cy, cz
        self.radius = radius
        self.u, self.vv = u, v
        self.thetaSpan = theta_end
        return arc_len >= 1e-4

    def _plan_pose(self, s):
        theta = self.thetaSpan*s
        ct, st = math.cos(theta), math.sin(theta)
        x = self.cx + self.radius*(ct*self.u[0] + st*self.vv[0])
        y = self.cy + self.radius*(ct*self.u[1] + st*self.vv[1])
        z = self.cz + self.radius*(ct*self.u[2] + st*self.vv[2])
        a, b, c = quat_to_rpy(slerp(self.q0, self.q1, s))
        return (x, y, z, a, b, c)

    def __call__(self, axis, Execute, PositionX, PositionY, PositionZ,
                 AuxX, AuxY, AuxZ, CirMode=0,
                 PositionA=0.0, PositionB=0.0, PositionC=0.0,
                 Speed=150.0, Accel=800.0, Jerk=8000.0,
                 MoveMode=0, BufferMode=0):
        tgt_deg = (PositionX, PositionY, PositionZ,
                   PositionA, PositionB, PositionC)
        rise = Execute and not self.exLast
        geom_ok = True
        if rise:
            ik_ok = ik(pose_rad(tgt_deg),
                       theta_ref=tuple(axis.CurTheta)) is not None
            # 预检：IK 可行且圆弧非退化（上升沿即拒非法指令）
            geom_ok = ik_ok and self._build_geom(axis, tgt_deg,
                                                 (AuxX, AuxY, AuxZ), CirMode)
            if geom_ok:
                self.tgtDeg = tgt_deg
            self._aux = (AuxX, AuxY, AuxZ)
            self._cirMode = CirMode

        wantStart = self._schedule(axis, Execute, Speed, Accel, Jerk,
                                   MoveMode, BufferMode, geom_ok)
        if wantStart is None:
            return

        if wantStart:
            self._take_token(axis, self.ACTIVE_TYPE)
            # 缓存提升发生在后续周期：圆弧起点必须按【实际提升时刻】的 Cur
            # 重建（rise 预检时的 Cur 是上一条指令途中的位姿）
            if not self._build_geom(axis, self.tgtDeg,
                                    self._aux, self._cirMode):
                self._emit_hold(axis, self.tgtDeg)
                self.s, self.phase = 1.0, 4
                arc_len = 0.0
            else:
                arc_len = self.radius*abs(self.thetaSpan)
            self.startDeg = axis_cur_pose(axis)
            sr = pose_rad(self.startDeg)
            tr = pose_rad(self.tgtDeg)
            self.q0 = rpy_to_quat(*sr[3:6])
            self.q1 = rpy_to_quat(*tr[3:6])
            drot = quat_angle(self.q0, self.q1)
            total = math.sqrt(arc_len*arc_len + (K_ANG*drot)**2)
            if total < 1e-4:
                self._emit_hold(axis, self.tgtDeg)
                self.s, self.phase = 1.0, 4
            else:
                self.maxSVel = Speed/total
                self.maxSAcc = Accel/total
                self.maxSJerk = Jerk/total
                self.s = self.sVel = self.sAcc = 0.0
                self.phase = 1

        self._check_abort(axis)

        if self.running and self.phase in (1, 2, 3):
            self._profile(axis, MoveMode)
            if self.phase != 4:
                pr = self._plan_pose(self.s)
                rates = fd_rates(self._plan_pose, self.s, self.sVel)
                self._emit_plan(axis, pr, rates)
            if self.phase == 4:
                self._emit_hold(axis, self.tgtDeg)

        if self.running and self.phase == 4:
            self._finish(axis)


# ============================================================
# FB_DeltaKinematics6（IK/FK/前馈 计算件）
# ============================================================
class FB_DeltaKinematics6:
    def __init__(self):
        self.reset()

    def reset(self):
        self.IK_OK = self.FK_OK = False
        self.tar = list(DEFAULT_REF)
        self.guess = list(DEFAULT_GUESS)
        self.cur = list(DEFAULT_GUESS)   # 弧度位姿
        self.cold = True                # 首拍/失能恢复：纯牛顿冷启动建立位姿

    def __call__(self, axis):
        P = PARAMS
        # ---- FK：6 反馈角 → 位姿 ----
        # 初值策略（六链 FK 有多组装配解，纯牛顿会在倾斜路径滑根，见 kinematics6
        # fk_track 注释）：
        #   · 冷启动：纯牛顿 + DEFAULT_GUESS（带姿态偏置）
        #   · Busy：以【规划位姿 Plan】为初值做 LM——电机实际位姿必在 Plan 的伺服
        #     滞后邻域内，Plan 所在的装配页即唯一正确页；prevCur 连续性跟踪在穿过
        #     FK 折叠奇异（vol≈1e-4 退化区）时会滑到另一页（实测姿态差 15°+）
        #   · 空闲：Plan 即上拍 Cur（核心空闲跟随），等价 prevCur 温启动
        th = tuple(axis.CurTheta)
        if self.cold:
            res = fk(th, guess=tuple(self.guess), P=P)
        else:
            guess_r = pose_rad((axis.PlanX, axis.PlanY, axis.PlanZ,
                                axis.PlanA, axis.PlanB, axis.PlanC))
            res = fk_track(th, guess=guess_r, P=P)
            if res is None:
                res = fk(th, guess=guess_r, P=P)   # 兜底纯牛顿
        if res is None:
            self.FK_OK = False
            axis.FkError = True
        else:
            self.FK_OK = True
            axis.FkError = False
            self.cur = list(res[0])
            self.guess = list(res[0])
            self.cold = False

        # ---- IK：规划位姿 → 6 目标角（闭式；失败整体保持）----
        plan_r = pose_rad((axis.PlanX, axis.PlanY, axis.PlanZ,
                           axis.PlanA, axis.PlanB, axis.PlanC))
        new_tar = ik(plan_r, theta_ref=tuple(self.tar), P=P)
        if new_tar is None:
            self.IK_OK = False
        else:
            self.IK_OK = True
            self.tar = list(new_tar)

        # ---- 前馈：解析关节速度（前馈失败轴严格保持）----
        dth = [0.0] * 6
        if axis.FfEnable and self.IK_OK:
            twist = (axis.PlanVX, axis.PlanVY, axis.PlanVZ,
                     math.radians(axis.PlanVA),
                     math.radians(axis.PlanVB),
                     math.radians(axis.PlanVC))
            vel = ik_vel(plan_r, tuple(self.tar), twist, P=P)
            for k in range(6):
                dth[k] = vel[k] * axis.CycleTime

        for k in range(6):
            axis.TarTheta[k] = self.tar[k] + axis.Kff * dth[k]


# ============================================================
# FB_DeltaCore6（纯运动学引擎）
# ============================================================
class FB_DeltaCore6:
    def __init__(self):
        self.fbKin = FB_DeltaKinematics6()

    def __call__(self, axis, Enable=True, FfEnable=True, Kff=0.5):
        if not Enable:
            axis.Enable = False
            axis.R, axis.r = PARAMS['R'], PARAMS['r']
            axis.L, axis.l = PARAMS['L'], PARAMS['l']
            axis.Rac, axis.ra = PARAMS['Rac'], PARAMS['ra']
            axis.La, axis.lb = PARAMS['La'], PARAMS['lb']
            axis.TarTheta = [0.0] * 6
            axis.CurX = axis.CurY = axis.CurZ = 0.0
            axis.CurA = axis.CurB = axis.CurC = 0.0
            axis.FkError = False
            axis.PlanX = axis.PlanY = axis.PlanZ = 0.0
            axis.PlanA = axis.PlanB = axis.PlanC = 0.0
            axis.PlanVX = axis.PlanVY = axis.PlanVZ = 0.0
            axis.PlanVA = axis.PlanVB = axis.PlanVC = 0.0
            axis.Busy = False
            axis.ActiveType = 0
            axis.owner = 0
            axis.cmdCount = 0
            axis.hasBuf = False
            axis.InPos = False
            self.fbKin.reset()
            return

        axis.R, axis.r = PARAMS['R'], PARAMS['r']
        axis.L, axis.l = PARAMS['L'], PARAMS['l']
        axis.Rac, axis.ra = PARAMS['Rac'], PARAMS['ra']
        axis.La, axis.lb = PARAMS['La'], PARAMS['lb']
        axis.CycleTime = CYCLE
        axis.FfEnable, axis.Kff = FfEnable, Kff
        axis.Enable = True

        if not axis.Busy:
            # 空闲：FK 算出的 Cur 直接作为规划，速度清零（先在 FK 后再置 Plan）
            pass

        self.fbKin(axis)

        # FK 结果回写轴
        cp = pose_deg(tuple(self.fbKin.cur))
        axis.CurX, axis.CurY, axis.CurZ = cp[0], cp[1], cp[2]
        axis.CurA, axis.CurB, axis.CurC = cp[3], cp[4], cp[5]

        if not axis.Busy:
            axis.PlanX, axis.PlanY, axis.PlanZ = axis.CurX, axis.CurY, axis.CurZ
            axis.PlanA, axis.PlanB, axis.PlanC = axis.CurA, axis.CurB, axis.CurC
            axis.PlanVX = axis.PlanVY = axis.PlanVZ = 0.0
            axis.PlanVA = axis.PlanVB = axis.PlanVC = 0.0
            # 空闲 1:1 跟随反馈角，角度无跳变
            for k in range(6):
                axis.TarTheta[k] = axis.CurTheta[k]


# ============================================================
# 测试仿真框架
# ============================================================
def ptp6(x, y, z, a=0.0, b=0.0, c=0.0, speed=200, accel=1000, jerk=10000,
         move_mode=0, buffer_mode=0):
    return dict(PositionX=x, PositionY=y, PositionZ=z,
                PositionA=a, PositionB=b, PositionC=c,
                Speed=speed, Accel=accel, Jerk=jerk,
                MoveMode=move_mode, BufferMode=buffer_mode)


def lin6(x, y, z, a=0.0, b=0.0, c=0.0, speed=200, accel=1000, jerk=10000,
         move_mode=0, buffer_mode=0):
    return ptp6(x, y, z, a, b, c, speed, accel, jerk, move_mode, buffer_mode)


def cir6(x, y, z, aux, a=0.0, b=0.0, c=0.0, cir_mode=0,
         speed=150, accel=800, jerk=8000, move_mode=0, buffer_mode=0):
    p = ptp6(x, y, z, a, b, c, speed, accel, jerk, move_mode, buffer_mode)
    p.update(dict(AuxX=aux[0], AuxY=aux[1], AuxZ=aux[2], CirMode=cir_mode))
    return p


class Sim6:
    """6 电机跟随仿真：每周期 写反馈→调指令FB→调核心→实际角度逼近目标。
    上电角度 = 零位 IK（不是全 0：θ=0 为臂水平的非装配姿态）。"""
    def __init__(self, axis, gain=0.3):
        self.axis = axis
        self.gain = gain
        home = ik(pose_rad(HOME_DEG))
        assert home is not None
        self.th = list(home)

    def step(self, core, calls, ff_enable=True, kff=0.5, enable=True):
        for k in range(6):
            self.axis.CurTheta[k] = self.th[k]
        for fb, ex, p in calls:
            fb(self.axis, Execute=ex, **p)
        core(self.axis, Enable=enable, FfEnable=ff_enable if enable else False,
             Kff=kff if enable else 0.0)
        for k in range(6):
            self.th[k] += (self.axis.TarTheta[k]-self.th[k])*self.gain


def run_fb(axis, core, sim, fb, p, exec_cycles=1, max_cycles=8000, settle=20):
    """驱动单个指令FB到完成，返回完成周期号(-1超时)"""
    for i in range(max_cycles):
        sim.step(core, [(fb, i < exec_cycles, p)])
        if i >= exec_cycles and fb.InPos and not fb.Busy:
            for _ in range(settle):
                sim.step(core, [(fb, False, p)])
            return i
    return -1


def pose_close(axis, xyz, abc=None, tol_xy=1.5, tol_ang=1.0):
    ok = abs(axis.PlanX-xyz[0]) < tol_xy and \
         abs(axis.PlanY-xyz[1]) < tol_xy and \
         abs(axis.PlanZ-xyz[2]) < tol_xy
    if abc is not None:
        ok = ok and abs(axis.PlanA-abc[0]) < tol_ang and \
             abs(axis.PlanB-abc[1]) < tol_ang and \
             abs(axis.PlanC-abc[2]) < tol_ang
    return ok


if __name__ == '__main__':
    passed = 0
    failed = 0

    def test(name, condition, detail=""):
        global passed, failed
        if condition:
            passed += 1
            print(f"  [PASS] {name}")
        else:
            failed += 1
            print(f"  [FAIL] {name}  {detail}")

    def setup():
        axis = DeltaAxis6()
        core = FB_DeltaCore6()
        sim = Sim6(axis)
        # 先空跑一拍使能核心（真实PLC：先使能核心再下指令）
        sim.step(core, [])
        return axis, core, sim

    # ============================================================
    print("=" * 60); print("测试 1: PTP 梯形曲线"); print("=" * 60)
    axis, core, sim = setup()
    fb = FB_PTP6()
    cycles = run_fb(axis, core, sim, fb, ptp6(30, 0, 300, speed=200, accel=1000))
    test("运动完成", cycles > 0, f"cycles={cycles}")
    test("到达目标 X≈30", pose_close(axis, (30, 0, 300)),
         f"({axis.PlanX:.2f},{axis.PlanY:.2f},{axis.PlanZ:.2f})")
    test("FB.InPos=TRUE", fb.InPos)
    test("FB.Busy=FALSE", not fb.Busy)
    test("轴空闲 ActiveType=0", axis.ActiveType == 0 and not axis.Busy)
    test("6 个电机角均为正(板下方)", all(axis.TarTheta[k] > 0 for k in range(6)))

    print("\n" + "=" * 60); print("测试 2: PTP S曲线（带姿态）"); print("=" * 60)
    axis, core, sim = setup()
    fb = FB_PTP6()
    cycles = run_fb(axis, core, sim, fb,
                    ptp6(-20, 25, 290, a=8, b=-5, c=10,
                         speed=150, accel=800, jerk=8000, move_mode=1))
    test("运动完成", cycles > 0)
    test("到达位姿(含姿态角)", pose_close(axis, (-20, 25, 290), (8, -5, 10)),
         f"({axis.PlanX:.1f},{axis.PlanY:.1f},{axis.PlanZ:.1f},"
         f"{axis.PlanA:.1f},{axis.PlanB:.1f},{axis.PlanC:.1f})")

    print("\n" + "=" * 60); print("测试 3: LIN 梯形曲线"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(0, 0, 300))
    fb = FB_LIN6()
    cycles = run_fb(axis, core, sim, fb, lin6(35, 20, 310, speed=200, accel=1000))
    test("运动完成", cycles > 0)
    test("到达目标", pose_close(axis, (35, 20, 310)),
         f"({axis.PlanX:.1f},{axis.PlanY:.1f},{axis.PlanZ:.1f})")

    print("\n" + "=" * 60); print("测试 4: LIN S曲线（带姿态 slerp）"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(0, 0, 300))
    fb = FB_LIN6()
    cycles = run_fb(axis, core, sim, fb,
                    lin6(-30, -20, 285, a=-6, b=8, c=-12,
                         speed=180, accel=900, jerk=9000, move_mode=1))
    test("运动完成", cycles > 0)
    test("到达目标", pose_close(axis, (-30, -20, 285), (-6, 8, -12)),
         f"({axis.PlanX:.1f},{axis.PlanY:.1f},{axis.PlanZ:.1f},"
         f"{axis.PlanA:.1f},{axis.PlanB:.1f},{axis.PlanC:.1f})")

    print("\n" + "=" * 60); print("测试 4b: LIN 路径直线度 + 姿态单调"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(0, 0, 300))
    fb = FB_LIN6()
    p = lin6(30, 30, 310, a=10, b=0, c=10, speed=80, accel=400)
    max_dev = 0.0
    prev_c = None
    mono = True
    for i in range(3000):
        sim.step(core, [(fb, i == 0, p)])
        if i > 0 and axis.Busy:
            # 点到直线(0,0,300)→(30,30,310) 的距离
            px, py, pz = axis.PlanX, axis.PlanY, axis.PlanZ
            t0 = (px*30 + py*30 + (pz-300)*10) / (30*30+30*30+100)
            t0 = min(max(t0, 0), 1)
            bx, by, bz = 30*t0, 30*t0, 300+10*t0
            max_dev = max(max_dev,
                          math.sqrt((px-bx)**2+(py-by)**2+(pz-bz)**2))
            if prev_c is not None and axis.PlanC < prev_c[1] - 0.01:
                mono = False
            prev_c = (axis.PlanA, axis.PlanC)
        if i > 0 and fb.InPos and not fb.Busy:
            break
    test("直线度偏差 < 0.2 mm", max_dev < 0.2, f"dev={max_dev:.4f}")
    test("姿态 yaw 沿程单调", mono)

    print("\n" + "=" * 60); print("测试 5: CIR 圆心式"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(30, 0, 300))
    fb = FB_CIR6()
    # 起点(30,0) 圆心(10,0) 半径20，+90° → 终点(10,20)
    cycles = run_fb(axis, core, sim, fb,
                    cir6(10, 20, 300, aux=(10, 0, 300), cir_mode=0))
    test("运动完成", cycles > 0)
    test("到达目标", pose_close(axis, (10, 20, 300), tol_xy=2.0),
         f"({axis.PlanX:.1f},{axis.PlanY:.1f},{axis.PlanZ:.1f})")

    print("\n" + "=" * 60); print("测试 5b: CIR 圆弧半径恒定"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(30, 0, 300))
    fb = FB_CIR6()
    p = cir6(10, 20, 300, aux=(10, 0, 300), speed=80, accel=400)
    max_rad_dev = 0.0
    for i in range(3000):
        sim.step(core, [(fb, i == 0, p)])
        if axis.Busy:
            rr = math.hypot(axis.PlanX-10, axis.PlanY-0)
            max_rad_dev = max(max_rad_dev, abs(rr-20))
        if i > 0 and fb.InPos and not fb.Busy:
            break
    test("圆弧半径偏差 < 0.2 mm", max_rad_dev < 0.2, f"dev={max_rad_dev:.4f}")

    print("\n" + "=" * 60); print("测试 6: CIR 三点式（带姿态）"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(30, 0, 300))
    fb = FB_CIR6()
    # 起点(30,0) 中间(10,20) 终点(-10,0)：圆心(10,0) 半径20 半圆弧
    cycles = run_fb(axis, core, sim, fb,
                    cir6(-10, 0, 300, a=0, b=0, c=12,
                         aux=(10, 20, 300), cir_mode=1))
    test("运动完成", cycles > 0)
    test("到达目标(含姿态)", pose_close(axis, (-10, 0, 300), (0, 0, 12),
                                         tol_xy=2.0, tol_ang=1.5),
         f"({axis.PlanX:.1f},{axis.PlanY:.1f},{axis.PlanZ:.1f},{axis.PlanC:.1f})")

    print("\n" + "=" * 60); print("测试 7: 拒绝模式 (BufferMode=0)"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(0, 0, 300))
    fb1, fb2 = FB_LIN6(), FB_LIN6()
    p1 = lin6(35, 0, 300, speed=25, accel=120, buffer_mode=0)
    p2 = lin6(0, 30, 300, speed=25, accel=120, buffer_mode=0)
    rejected_seen = False
    for i in range(400):
        calls = [(fb1, i == 0, p1), (fb2, False, p2)]
        if i == 100:
            calls[1] = (fb2, True, p2)
        sim.step(core, calls)
        if i == 100:
            rejected_seen = fb2.Rejected
    test("fb1 运动中 Busy=TRUE", fb1.Busy)
    test("fb2 Rejected=TRUE", rejected_seen, "第二条指令应被拒绝")
    test("fb2 未启动 Busy=FALSE", not fb2.Busy)
    test("ActiveType=2(LIN)", axis.ActiveType == 2, f"ActiveType={axis.ActiveType}")

    print("\n" + "=" * 60); print("测试 8: 打断模式 (BufferMode=1)"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(0, 0, 300))
    fb1, fb2 = FB_LIN6(), FB_PTP6()
    p1 = lin6(35, 0, 300, speed=25, accel=120, buffer_mode=1)
    p2 = ptp6(0, 30, 315, b=6, speed=200, accel=1000, buffer_mode=1)
    interrupted = False
    for i in range(2500):
        calls = [(fb1, i == 0, p1), (fb2, False, p2)]
        if i == 150:
            calls[1] = (fb2, True, p2)
            interrupted = True
        sim.step(core, calls)
        if i > 150 and fb2.InPos and not fb2.Busy:
            break
    test("打断已触发", interrupted)
    test("fb1 CommandAborted=TRUE", fb1.CommandAborted,
         f"aborted={fb1.CommandAborted}")
    test("打断后到达新目标 Y≈30", pose_close(axis, (0, 30, 315), (0, 6, 0),
                                             tol_xy=2, tol_ang=1.5),
         f"({axis.PlanY:.1f},{axis.PlanZ:.1f},B={axis.PlanB:.1f})")

    print("\n" + "=" * 60); print("测试 9: 缓存模式 (BufferMode=2)"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(0, 0, 300))
    fb1, fb2 = FB_LIN6(), FB_LIN6()
    p1 = lin6(35, 0, 300, speed=60, accel=300, buffer_mode=2)
    p2 = lin6(35, 25, 300, speed=60, accel=300, buffer_mode=2)
    buf_sent = False
    both_done = False
    for i in range(6000):
        calls = [(fb1, i == 0, p1), (fb2, False, p2)]
        if i == 5:
            calls[1] = (fb2, True, p2)
            buf_sent = True
        sim.step(core, calls)
        if buf_sent and fb2.InPos and not fb2.Busy and abs(axis.PlanY-25) < 2:
            both_done = True
            break
    test("缓存指令已发送", buf_sent)
    test("两条都完成且到达第二条目标", both_done, f"PlanY={axis.PlanY:.1f}")

    print("\n" + "=" * 60); print("测试 10: 缓存槽满拒绝"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(0, 0, 300))
    fb1, fb2, fb3 = FB_LIN6(), FB_LIN6(), FB_LIN6()
    p1 = lin6(35, 0, 300, speed=25, accel=120, buffer_mode=2)
    p2 = lin6(0, 30, 300, speed=25, accel=120, buffer_mode=2)
    p3 = lin6(20, 20, 300, speed=25, accel=120, buffer_mode=2)
    rej1 = rej2 = buf_full = False
    for i in range(400):
        calls = [(fb1, i == 0, p1), (fb2, False, p2), (fb3, False, p3)]
        if i == 5:
            calls[1] = (fb2, True, p2)
        if i == 10:
            calls[2] = (fb3, True, p3)
        sim.step(core, calls)
        if i == 5:
            rej1 = fb2.Rejected
        if i == 10:
            rej2 = fb3.Rejected
            buf_full = axis.hasBuf
            break
    test("第一条缓存成功(不拒绝)", not rej1)
    test("第二条缓存被拒绝(槽满)", rej2)
    test("axis.hasBuf=TRUE", buf_full)

    print("\n" + "=" * 60); print("测试 11: 打断清空缓存"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(0, 0, 300))
    fb1, fb2, fb3 = FB_LIN6(), FB_LIN6(), FB_PTP6()
    p1 = lin6(35, 0, 300, speed=25, accel=120, buffer_mode=2)
    p2 = lin6(0, 30, 300, speed=25, accel=120, buffer_mode=2)
    p3 = ptp6(0, 0, 315, speed=200, accel=1000, buffer_mode=1)
    buf_waiting_aborted = False
    for i in range(500):
        calls = [(fb1, i == 0, p1), (fb2, False, p2), (fb3, False, p3)]
        if i == 5:
            calls[1] = (fb2, True, p2)
        if i == 10:
            calls[2] = (fb3, True, p3)
        sim.step(core, calls)
        if i >= 11 and fb2.CommandAborted and not axis.hasBuf:
            buf_waiting_aborted = True
            break
    test("缓存等待者收到 CommandAborted", buf_waiting_aborted)
    test("打断后 hasBuf=FALSE", not axis.hasBuf)
    test("打断者 fb3 获得轴", fb3.Busy and axis.ActiveType == 1)

    print("\n" + "=" * 60); print("测试 12: 零距离运动"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(20, 20, 300, a=5, b=0, c=5))
    fb = FB_LIN6()
    cycles = run_fb(axis, core, sim, fb,
                    lin6(20, 20, 300, a=5, b=0, c=5), max_cycles=100)
    test("零距离立即完成", 0 <= cycles < 5, f"cycles={cycles}")
    test("InPos=TRUE", fb.InPos)

    print("\n" + "=" * 60); print("测试 13: 速度前馈使能 vs 禁用"); print("=" * 60)
    # 注：6DOF 上电即在零位（零位 IK 角度），目标必须离开零位才能激励前馈
    p = ptp6(30, 0, 300, speed=300, accel=2000)
    axis1, core1, sim1 = setup()
    fb1 = FB_PTP6()
    for i in range(60):
        sim1.step(core1, [(fb1, i == 0, p)], ff_enable=False, kff=0.0)
    tar_noff = axis1.TarTheta[0]
    axis2, core2, sim2 = setup()
    fb2 = FB_PTP6()
    for i in range(60):
        sim2.step(core2, [(fb2, i == 0, p)], ff_enable=True, kff=0.5)
    tar_ff = axis2.TarTheta[0]
    test("前馈使能时主1目标角不同", abs(tar_ff-tar_noff) > 1e-6,
         f"noff={tar_noff:.6f} ff={tar_ff:.6f}")
    # 6DOF：副电机前馈同样应生效
    tar_noff4 = axis1.TarTheta[3]
    tar_ff4 = axis2.TarTheta[3]
    test("前馈使能时副1目标角不同", abs(tar_ff4-tar_noff4) > 1e-6,
         f"noff={tar_noff4:.6f} ff={tar_ff4:.6f}")

    print("\n" + "=" * 60); print("测试 14: 连续 LIN 运动无跳变"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(0, 0, 300))
    fb = FB_LIN6()
    pts = [(25, 0, 300, 0, 0, 0), (25, 25, 305, 5, 5, 0), (0, 25, 300, 0, 0, 8)]
    all_ok = True
    prev_tar = list(axis.TarTheta)
    max_jump = 0.0
    for pt0 in pts:
        cycles = run_fb(axis, core, sim, fb,
                        lin6(pt0[0], pt0[1], pt0[2], pt0[3], pt0[4], pt0[5],
                             speed=120, accel=700))
        if cycles < 0 or not pose_close(axis, pt0[0:3], pt0[3:6]):
            all_ok = False
            break
    test("3段连续直线(含姿态)都到达", all_ok)

    print("\n" + "=" * 60); print("测试 15: PTP 大行程 S曲线（含姿态）"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(0, 0, 320))
    fb = FB_PTP6()
    cycles = run_fb(axis, core, sim, fb,
                    ptp6(-30, 25, 285, a=-8, b=8, c=-15,
                         speed=200, accel=1500, move_mode=1),
                    max_cycles=8000)
    test("大行程完成", cycles > 0, f"cycles={cycles}")
    test("到达目标(含姿态)",
         pose_close(axis, (-30, 25, 285), (-8, 8, -15), tol_xy=2, tol_ang=1.5),
         f"({axis.PlanX:.1f},{axis.PlanY:.1f},{axis.PlanZ:.1f},"
         f"{axis.PlanA:.1f},{axis.PlanB:.1f},{axis.PlanC:.1f})")

    print("\n" + "=" * 60); print("测试 16: 纯姿态运动（位置不动）"); print("=" * 60)
    axis, core, sim = setup()
    fb = FB_LIN6()
    cycles = run_fb(axis, core, sim, fb,
                    lin6(0, 0, 300, a=10, b=-8, c=15, speed=60, accel=300))
    test("纯姿态运动完成", cycles > 0)
    test("位置保持零位", pose_close(axis, (0, 0, 300), tol_xy=0.5),
         f"({axis.PlanX:.2f},{axis.PlanY:.2f},{axis.PlanZ:.2f})")
    test("姿态到达", pose_close(axis, (0, 0, 300), (10, -8, 15), tol_xy=0.5))
    test("运动中主/副电机均有动作",
         max(abs(axis.TarTheta[k]-sim.th[k]) for k in range(6)) >= 0
         and cycles > 0)

    print("\n" + "=" * 60); print("测试 17: IK 预检（不可达位姿上升沿 Rejected）"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(0, 0, 300))
    fb = FB_PTP6()
    p_bad = ptp6(300, 0, 300, buffer_mode=0)   # ρ=300 远超物理边界
    sim.step(core, [(fb, True, p_bad)])
    test("不可达目标 Rejected", fb.Rejected and not fb.Busy and not axis.Busy)
    sim.step(core, [(fb, False, p_bad)])
    test("Rejected 单周期脉冲", not fb.Rejected)
    fb2 = FB_LIN6()
    p_ok = lin6(20, 0, 300, buffer_mode=0)
    for i in range(2000):
        sim.step(core, [(fb2, i == 0, p_ok)])
        if i > 0 and fb2.InPos and not fb2.Busy:
            break
    test("拒绝后轴仍可正常接单", fb2.InPos and pose_close(axis, (20, 0, 300)))

    print("\n" + "=" * 60); print("测试 18: 失能/恢复"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP6(), ptp6(0, 0, 300))
    fv = FB_LIN6()
    p = lin6(35, 0, 300, speed=25, accel=120)
    for i in range(150):
        sim.step(core, [(fv, i == 0, p)])
    moving = fv.Busy and axis.Busy
    for _ in range(5):
        sim.step(core, [(fv, False, p)], enable=False)
    fb_reset = not fv.Busy and not fv.InPos and not axis.Busy
    out_clear = all(axis.TarTheta[k] == 0 for k in range(6)) and \
        axis.CurX == 0 and axis.CurA == 0
    # 反馈回零位后重新使能
    sim.th = list(ik(pose_rad(HOME_DEG)))
    for _ in range(3):
        sim.step(core, [])
    f2 = FB_PTP6()
    cycles = run_fb(axis, core, sim, f2, ptp6(20, -15, 310, buffer_mode=0))
    test("失能前运动中", moving)
    test("失能后指令FB复位", fb_reset)
    test("失能后6轴输出清零", out_clear)
    test("重新使能后新指令正常执行",
         cycles > 0 and f2.InPos and pose_close(axis, (20, -15, 310)))
    test("失能期间遗留FB不会自行复活", not fv.running and fv.phase == 0)

    # ============================================================
    print("\n" + "=" * 60)
    print(f"测试结果: {passed} 通过, {failed} 失败")
    print("=" * 60)
    raise SystemExit(1 if failed else 0)
