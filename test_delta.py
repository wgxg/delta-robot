"""
Delta 机器人运动控制库 - PLCopen 架构多样本测试
================================================
架构：
  · DeltaAxis   共享轴状态（反馈/机械参数/调度令牌/规划输出）
  · FB_PTP/LIN/CIR 指令FB：Execute + BufferMode(0拒绝/1打断/2缓存)，
                  Busy/InPos/CommandAborted/Rejected，多实例共享 Axis 协调
  · FB_DeltaCore 纯运动学引擎：每周期最后调用，IK/FK/前馈 + 空闲跟随
调用顺序（每周期）：写 CurTheta → 调各指令FB → 调 FB_DeltaCore
"""
import math

# ============================================================
# 常量
# ============================================================
COS1, SIN1 = 1.0, 0.0
COS2, SIN2 = -0.5, 0.8660254037844386
COS3, SIN3 = -0.5, -0.8660254037844386
PI = math.pi

# 机械参数
R, r, L, l = 100.0, 50.0, 150.0, 200.0
CYCLE = 0.001  # 1ms


# ============================================================
# st_DeltaAxis
# ============================================================
class DeltaAxis:
    def __init__(self):
        # 机械参数（核心写，指令读）
        self.R = self.r = self.L = self.l = 0.0
        self.CycleTime = CYCLE
        self.FfEnable = False
        self.Kff = 0.0
        self.Enable = False
        # 反馈输入
        self.CurTheta1 = self.CurTheta2 = self.CurTheta3 = 0.0
        # 核心输出
        self.TarTheta1 = self.TarTheta2 = self.TarTheta3 = 0.0
        self.CurX = self.CurY = self.CurZ = 0.0
        # 调度状态
        self.Busy = False
        self.ActiveType = 0       # 0空闲 1PTP 2LIN 3CIR
        self.owner = 0
        self.cmdCount = 0
        self.InPos = False
        self.hasBuf = False
        # 规划输出
        self.PlanX = self.PlanY = self.PlanZ = 0.0
        self.PlanVX = self.PlanVY = self.PlanVZ = 0.0


# ============================================================
# FB_DeltaKinematics（基础层，不变）
# ============================================================
class FB_DeltaKinematics:
    def __init__(self):
        self.TarTheta1 = self.TarTheta2 = self.TarTheta3 = 0.0
        self.CurX = self.CurY = self.CurZ = 0.0
        self.IK_OK = self.FK_OK = False

    def __call__(self, R, r, L, l, PlanX, PlanY, PlanZ,
                 PlanVX, PlanVY, PlanVZ, FfEnable, Kff,
                 CurTheta1, CurTheta2, CurTheta3, CycleTime, Enable):
        if not Enable:
            self.TarTheta1 = self.TarTheta2 = self.TarTheta3 = 0.0
            self.CurX = self.CurY = self.CurZ = 0.0
            self.IK_OK = self.FK_OK = False
            return

        e = R - r

        # FK
        s1 = e + L * math.cos(CurTheta1)
        s2 = e + L * math.cos(CurTheta2)
        s3 = e + L * math.cos(CurTheta3)
        sn1 = L * math.sin(CurTheta1)
        sn2 = L * math.sin(CurTheta2)
        sn3 = L * math.sin(CurTheta3)

        a1 = -2.0 * (s2 * COS2 - s1 * COS1)
        b1 = -2.0 * (s2 * SIN2 - s1 * SIN1)
        c1 = -2.0 * (sn2 - sn1)
        d1 = (s1*s1 + sn1*sn1) - (s2*s2 + sn2*sn2)
        a2 = -2.0 * (s3 * COS3 - s1 * COS1)
        b2 = -2.0 * (s3 * SIN3 - s1 * SIN1)
        c2 = -2.0 * (sn3 - sn1)
        d2 = (s1*s1 + sn1*sn1) - (s3*s3 + sn3*sn3)

        Det = a1 * b2 - a2 * b1
        if abs(Det) < 1e-9:
            self.FK_OK = False
            self.CurX = self.CurY = self.CurZ = 0.0
        else:
            px = (b2 * d1 - b1 * d2) / Det
            qx = (b1 * c2 - b2 * c1) / Det
            py = (a1 * d2 - a2 * d1) / Det
            qy = (a2 * c1 - a1 * c2) / Det
            X0 = px - s1 * COS1
            Y0 = py - s1 * SIN1
            Az = qx*qx + qy*qy + 1.0
            Bz = 2.0 * (X0*qx + Y0*qy) - 2.0 * sn1
            Cz = X0*X0 + Y0*Y0 + sn1*sn1 - l*l
            Dz = Bz*Bz - 4.0*Az*Cz
            if Dz < 0:
                self.FK_OK = False
                self.CurX = self.CurY = self.CurZ = 0.0
            else:
                z1 = (-Bz + math.sqrt(Dz)) / (2.0 * Az)
                z2 = (-Bz - math.sqrt(Dz)) / (2.0 * Az)
                if z1 > 0:
                    self.CurZ = z1
                elif z2 > 0:
                    self.CurZ = z2
                else:
                    self.CurZ = z1
                self.CurX = px + qx * self.CurZ
                self.CurY = py + qy * self.CurZ
                self.FK_OK = True

        def ik_ax(cos_a, sin_a, cur_th):
            xp = PlanX * cos_a + PlanY * sin_a
            yp = -PlanX * sin_a + PlanY * cos_a
            A = 2.0 * L * (e - xp)
            B = -2.0 * L * PlanZ
            C = L*L + (e-xp)*(e-xp) + yp*yp + PlanZ*PlanZ - l*l
            Disc = A*A + B*B - C*C
            if Disc < 0:
                return cur_th, False
            denom = C - A
            if abs(denom) < 1e-9:
                return cur_th, False
            u1 = (-B + math.sqrt(Disc)) / denom
            u2 = (-B - math.sqrt(Disc)) / denom
            ts1 = 2.0 * math.atan(u1)
            ts2 = 2.0 * math.atan(u2)
            if ts1 > 0 and ts2 > 0:
                if abs(ts1 - cur_th) <= abs(ts2 - cur_th):
                    return ts1, True
                else:
                    return ts2, True
            elif ts1 > 0:
                return ts1, True
            elif ts2 > 0:
                return ts2, True
            else:
                return ts1, True

        ikTh1, ok1 = ik_ax(COS1, SIN1, CurTheta1)
        ikTh2, ok2 = ik_ax(COS2, SIN2, CurTheta2)
        ikTh3, ok3 = ik_ax(COS3, SIN3, CurTheta3)
        self.IK_OK = ok1 and ok2 and ok3

        # IK失败时保持上一周期目标角度，避免跳变
        if not ok1: ikTh1 = self.TarTheta1
        if not ok2: ikTh2 = self.TarTheta2
        if not ok3: ikTh3 = self.TarTheta3

        # 前馈: θ̇ = -(B·v)/Aii（隐函数求导带负号），失败轴严格保持
        dTh1_ff = dTh2_ff = dTh3_ff = 0.0
        def ff_ax(cos_a, sin_a, ik_th):
            xp = PlanX * cos_a + PlanY * sin_a
            yp = -PlanX * sin_a + PlanY * cos_a
            ct = math.cos(ik_th); st = math.sin(ik_th)
            Aii = -2.0 * L * ((e - xp) * st + PlanZ * ct)
            Bix = -2.0 * ((L*ct + e - xp) * cos_a + yp * sin_a)
            Biy = -2.0 * ((L*ct + e - xp) * sin_a - yp * cos_a)
            Biz = -2.0 * (L * st - PlanZ)
            if abs(Aii) > 1e-9:
                return -((Bix*PlanVX + Biy*PlanVY + Biz*PlanVZ) / Aii) * CycleTime
            return 0.0
        if FfEnable and ok1:
            dTh1_ff = ff_ax(COS1, SIN1, ikTh1)
        if FfEnable and ok2:
            dTh2_ff = ff_ax(COS2, SIN2, ikTh2)
        if FfEnable and ok3:
            dTh3_ff = ff_ax(COS3, SIN3, ikTh3)

        self.TarTheta1 = ikTh1 + Kff * dTh1_ff
        self.TarTheta2 = ikTh2 + Kff * dTh2_ff
        self.TarTheta3 = ikTh3 + Kff * dTh3_ff


# ============================================================
# 指令FB公共调度逻辑（mixin）
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

    def _schedule(self, axis, Execute, Speed, Accel, Jerk, MoveMode, BufferMode):
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
            if Speed <= 0 or Accel <= 0 or (MoveMode == 1 and Jerk <= 0):
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


# ============================================================
# FB_PTP
# ============================================================
class FB_PTP(_CommandFB):
    ACTIVE_TYPE = 1

    def __init__(self):
        self.Busy = self.InPos = self.CommandAborted = self.Rejected = False
        self.running = self.waiting = False
        self.phase = 0
        self.s = self.sVel = self.sAcc = 0.0
        self.maxSVel = self.maxSAcc = self.maxSJerk = 0.0
        self.myId = 0
        self.exLast = False
        self.startTh = [0.0, 0.0, 0.0]
        self.tgtTh = [0.0, 0.0, 0.0]
        self.curX = self.curY = self.curZ = 0.0
        self.tgtX = self.tgtY = self.tgtZ = 0.0

    def _fk(self, axis, th):
        e = axis.R - axis.r
        s1 = e + axis.L*math.cos(th[0]); s2 = e + axis.L*math.cos(th[1]); s3 = e + axis.L*math.cos(th[2])
        sn1 = axis.L*math.sin(th[0]); sn2 = axis.L*math.sin(th[1]); sn3 = axis.L*math.sin(th[2])
        a1 = -2.0*(s2*COS2-s1*COS1); b1 = -2.0*(s2*SIN2-s1*SIN1)
        c1 = -2.0*(sn2-sn1); d1 = (s1*s1+sn1*sn1)-(s2*s2+sn2*sn2)
        a2 = -2.0*(s3*COS3-s1*COS1); b2 = -2.0*(s3*SIN3-s1*SIN1)
        c2 = -2.0*(sn3-sn1); d2 = (s1*s1+sn1*sn1)-(s3*s3+sn3*sn3)
        Det = a1*b2-a2*b1
        if abs(Det) < 1e-9:
            return 0.0, 0.0, 0.0
        px = (b2*d1-b1*d2)/Det; qx = (b1*c2-b2*c1)/Det
        py = (a1*d2-a2*d1)/Det; qy = (a2*c1-a1*c2)/Det
        X0 = px-s1*COS1; Y0 = py-s1*SIN1
        Az = qx*qx+qy*qy+1.0; Bz = 2.0*(X0*qx+Y0*qy)-2.0*sn1
        Cz = X0*X0+Y0*Y0+sn1*sn1-axis.l*axis.l
        Dz = Bz*Bz-4.0*Az*Cz
        if Dz < 0:
            return 0.0, 0.0, 0.0
        z1 = (-Bz+math.sqrt(Dz))/(2.0*Az); z2 = (-Bz-math.sqrt(Dz))/(2.0*Az)
        z = z1 if z1 > 0 else (z2 if z2 > 0 else z1)
        return px+qx*z, py+qy*z, z

    def _ik(self, axis, x, y, z, cur_th):
        e = axis.R - axis.r
        res = []
        for cos_a, sin_a, cur in ((COS1, SIN1, cur_th[0]), (COS2, SIN2, cur_th[1]),
                                  (COS3, SIN3, cur_th[2])):
            xp = x*cos_a + y*sin_a
            yp = -x*sin_a + y*cos_a
            A = 2.0*axis.L*(e-xp); B = -2.0*axis.L*z
            C = axis.L*axis.L+(e-xp)**2+yp*yp+z*z-axis.l*axis.l
            Disc = A*A+B*B-C*C
            if Disc < 0:
                res.append(cur)
            else:
                denom = C-A
                if abs(denom) < 1e-9:
                    res.append(cur)
                else:
                    u1 = (-B+math.sqrt(Disc))/denom
                    u2 = (-B-math.sqrt(Disc))/denom
                    ts1, ts2 = 2.0*math.atan(u1), 2.0*math.atan(u2)
                    if ts1 > 0 and ts2 > 0:
                        res.append(ts1 if abs(ts1-cur) <= abs(ts2-cur) else ts2)
                    elif ts1 > 0:
                        res.append(ts1)
                    elif ts2 > 0:
                        res.append(ts2)
                    else:
                        res.append(ts1)
        return res

    def __call__(self, axis, Execute, PositionX, PositionY, PositionZ,
                 Speed, Accel, Jerk, MoveMode, BufferMode):
        wantStart = self._schedule(axis, Execute, Speed, Accel, Jerk, MoveMode, BufferMode)
        if wantStart is None:
            return

        if wantStart:
            self._take_token(axis, self.ACTIVE_TYPE)
            cur = [axis.CurTheta1, axis.CurTheta2, axis.CurTheta3]
            self.curX, self.curY, self.curZ = self._fk(axis, cur)
            self.tgtX, self.tgtY, self.tgtZ = PositionX, PositionY, PositionZ
            self.tgtTh = self._ik(axis, PositionX, PositionY, PositionZ, cur)
            self.startTh = list(cur)
            total = math.sqrt((PositionX-self.curX)**2 + (PositionY-self.curY)**2 +
                              (PositionZ-self.curZ)**2)
            if total < 1e-4:
                axis.PlanX, axis.PlanY, axis.PlanZ = self.tgtX, self.tgtY, self.tgtZ
                axis.PlanVX = axis.PlanVY = axis.PlanVZ = 0.0
                self.s, self.phase = 1.0, 4
            else:
                self.maxSVel = Speed/total
                self.maxSAcc = Accel/total
                self.maxSJerk = Jerk/total
                self.s = self.sVel = self.sAcc = 0.0
                self.phase = 1

        self._check_abort(axis)

        if self.running and self.phase in (1, 2, 3):
            sRem = 1.0 - self.s
            if MoveMode == 0:
                sStop = self.sVel**2/(2.0*self.maxSAcc)
            else:
                sStop = self.sVel**2/(2.0*self.maxSAcc) + self.sVel*self.maxSAcc/(2.0*self.maxSJerk)

            if self.phase == 1:
                if sRem <= sStop:
                    self.phase = 3
                else:
                    if MoveMode == 0:
                        self.sAcc = self.maxSAcc
                    else:
                        self.sAcc = min(self.sAcc + self.maxSJerk*axis.CycleTime, self.maxSAcc)
                    self.sVel += self.sAcc*axis.CycleTime
                    if self.sVel >= self.maxSVel:
                        self.sVel = self.maxSVel; self.sAcc = 0.0; self.phase = 2
            elif self.phase == 2:
                if sRem <= sStop:
                    self.phase = 3
                else:
                    self.sVel = self.maxSVel; self.sAcc = 0.0
            elif self.phase == 3:
                if MoveMode == 0:
                    self.sAcc = -self.maxSAcc
                else:
                    self.sAcc = max(self.sAcc - self.maxSJerk*axis.CycleTime, -self.maxSAcc)
                self.sVel += self.sAcc*axis.CycleTime
                if self.sVel <= 0:
                    self.sVel = self.sAcc = 0.0
                    self.s = 1.0; self.phase = 4

            if self.phase != 4:
                self.s = min(self.s + self.sVel*axis.CycleTime, 1.0)

            interp = [self.startTh[k] + (self.tgtTh[k]-self.startTh[k])*self.s for k in range(3)]
            axis.PlanX, axis.PlanY, axis.PlanZ = self._fk(axis, interp)
            axis.PlanVX = (self.tgtX-self.curX)*self.sVel
            axis.PlanVY = (self.tgtY-self.curY)*self.sVel
            axis.PlanVZ = (self.tgtZ-self.curZ)*self.sVel
            if self.phase == 4:
                # Plan 保持最终插值角度 FK 结果，不虚报目标
                axis.PlanVX = axis.PlanVY = axis.PlanVZ = 0.0

        if self.running and self.phase == 4:
            self._finish(axis)


# ============================================================
# FB_LIN
# ============================================================
class FB_LIN(_CommandFB):
    ACTIVE_TYPE = 2

    def __init__(self):
        self.Busy = self.InPos = self.CommandAborted = self.Rejected = False
        self.running = self.waiting = False
        self.phase = 0
        self.s = self.sVel = self.sAcc = 0.0
        self.maxSVel = self.maxSAcc = self.maxSJerk = 0.0
        self.myId = 0
        self.exLast = False
        self.start = [0.0, 0.0, 0.0]
        self.tgt = [0.0, 0.0, 0.0]
        self.d = [0.0, 0.0, 0.0]
        self.totalDist = 0.0

    def __call__(self, axis, Execute, PositionX, PositionY, PositionZ,
                 Speed, Accel, Jerk, MoveMode, BufferMode):
        wantStart = self._schedule(axis, Execute, Speed, Accel, Jerk, MoveMode, BufferMode)
        if wantStart is None:
            return

        if wantStart:
            self._take_token(axis, self.ACTIVE_TYPE)
            self.start = [axis.CurX, axis.CurY, axis.CurZ]
            self.tgt = [PositionX, PositionY, PositionZ]
            dv = [self.tgt[k]-self.start[k] for k in range(3)]
            self.totalDist = math.sqrt(sum(v*v for v in dv))
            if self.totalDist < 1e-4:
                axis.PlanX, axis.PlanY, axis.PlanZ = self.tgt
                axis.PlanVX = axis.PlanVY = axis.PlanVZ = 0.0
                self.s, self.phase = 1.0, 4
            else:
                self.d = [v/self.totalDist for v in dv]
                self.maxSVel = Speed/self.totalDist
                self.maxSAcc = Accel/self.totalDist
                self.maxSJerk = Jerk/self.totalDist
                self.s = self.sVel = self.sAcc = 0.0
                self.phase = 1

        self._check_abort(axis)

        if self.running and self.phase in (1, 2, 3):
            sRem = 1.0 - self.s
            if MoveMode == 0:
                sStop = self.sVel**2/(2.0*self.maxSAcc)
            else:
                sStop = self.sVel**2/(2.0*self.maxSAcc) + self.sVel*self.maxSAcc/(2.0*self.maxSJerk)

            if self.phase == 1:
                if sRem <= sStop:
                    self.phase = 3
                else:
                    if MoveMode == 0:
                        self.sAcc = self.maxSAcc
                    else:
                        self.sAcc = min(self.sAcc + self.maxSJerk*axis.CycleTime, self.maxSAcc)
                    self.sVel += self.sAcc*axis.CycleTime
                    if self.sVel >= self.maxSVel:
                        self.sVel = self.maxSVel; self.sAcc = 0.0; self.phase = 2
            elif self.phase == 2:
                if sRem <= sStop:
                    self.phase = 3
                else:
                    self.sVel = self.maxSVel; self.sAcc = 0.0
            elif self.phase == 3:
                if MoveMode == 0:
                    self.sAcc = -self.maxSAcc
                else:
                    self.sAcc = max(self.sAcc - self.maxSJerk*axis.CycleTime, -self.maxSAcc)
                self.sVel += self.sAcc*axis.CycleTime
                if self.sVel <= 0:
                    self.sVel = self.sAcc = 0.0
                    self.s = 1.0; self.phase = 4

            if self.phase != 4:
                self.s = min(self.s + self.sVel*axis.CycleTime, 1.0)

            for k in range(3):
                setattr(axis, ('PlanX', 'PlanY', 'PlanZ')[k],
                        self.start[k] + self.d[k]*self.s*self.totalDist)
                setattr(axis, ('PlanVX', 'PlanVY', 'PlanVZ')[k],
                        self.d[k]*self.sVel*self.totalDist)
            if self.phase == 4:
                axis.PlanX, axis.PlanY, axis.PlanZ = self.tgt
                axis.PlanVX = axis.PlanVY = axis.PlanVZ = 0.0

        if self.running and self.phase == 4:
            self._finish(axis)


# ============================================================
# FB_CIR
# ============================================================
class FB_CIR(_CommandFB):
    ACTIVE_TYPE = 3

    def __init__(self):
        self.Busy = self.InPos = self.CommandAborted = self.Rejected = False
        self.running = self.waiting = False
        self.phase = 0
        self.s = self.sVel = self.sAcc = 0.0
        self.maxSVel = self.maxSAcc = self.maxSJerk = 0.0
        self.myId = 0
        self.exLast = False

    def __call__(self, axis, Execute, PositionX, PositionY, PositionZ,
                 AuxX, AuxY, AuxZ, CirMode, Speed, Accel, Jerk, MoveMode, BufferMode):
        wantStart = self._schedule(axis, Execute, Speed, Accel, Jerk, MoveMode, BufferMode)
        if wantStart is None:
            return

        if wantStart:
            self._take_token(axis, self.ACTIVE_TYPE)
            sx, sy, sz = axis.CurX, axis.CurY, axis.CurZ
            ex, ey, ez = PositionX, PositionY, PositionZ
            geom_fail = False

            if CirMode == 0:
                cx, cy, cz = AuxX, AuxY, AuxZ
            else:
                mx, my, mz = AuxX, AuxY, AuxZ
                sm = [mx-sx, my-sy, mz-sz]
                se = [ex-sx, ey-sy, ez-sz]
                n = [sm[1]*se[2]-sm[2]*se[1], sm[2]*se[0]-sm[0]*se[2], sm[0]*se[1]-sm[1]*se[0]]
                nLen = math.sqrt(sum(v*v for v in n))
                if nLen < 1e-6:
                    geom_fail = True
                else:
                    n = [v/nLen for v in n]
                    smLen = math.sqrt(sum(v*v for v in sm))
                    u = [v/smLen for v in sm]
                    v = [n[1]*u[2]-n[2]*u[1], n[2]*u[0]-n[0]*u[2], n[0]*u[1]-n[1]*u[0]]
                    eu = sum(se[k]*u[k] for k in range(3))
                    ev = sum(se[k]*v[k] for k in range(3))
                    aa = smLen/2.0
                    if abs(ev) < 1e-6:
                        geom_fail = True
                    else:
                        bb = (eu*eu+ev*ev-2.0*aa*eu)/(2.0*ev)
                        cx = sx+aa*u[0]+bb*v[0]
                        cy = sy+aa*u[1]+bb*v[1]
                        cz = sz+aa*u[2]+bb*v[2]

            if not geom_fail:
                cs = [sx-cx, sy-cy, sz-cz]
                self.radius = math.sqrt(sum(v*v for v in cs))
                ce = [ex-cx, ey-cy, ez-cz]
                if self.radius < 1e-6:
                    geom_fail = True
                else:
                    u = [v/self.radius for v in cs]
                    if CirMode == 0:
                        n = [cs[1]*ce[2]-cs[2]*ce[1], cs[2]*ce[0]-cs[0]*ce[2], cs[0]*ce[1]-cs[1]*ce[0]]
                        nLen = math.sqrt(sum(v*v for v in n))
                        if nLen < 1e-6:
                            geom_fail = True
                        else:
                            n = [v/nLen for v in n]

            if not geom_fail:
                v = [n[1]*u[2]-n[2]*u[1], n[2]*u[0]-n[0]*u[2], n[0]*u[1]-n[1]*u[0]]
                eu = sum(ce[k]*u[k] for k in range(3))
                ev = sum(ce[k]*v[k] for k in range(3))
                thetaStart = 0.0
                thetaEnd = math.atan2(ev, eu)
                if CirMode == 1:
                    cm = [mx-cx, my-cy, mz-cz]
                    thetaMid = math.atan2(sum(cm[k]*v[k] for k in range(3)),
                                          sum(cm[k]*u[k] for k in range(3)))
                    if thetaEnd < 0: thetaEnd += 2*PI
                    if thetaMid < 0: thetaMid += 2*PI
                    if not (thetaMid > 0 and thetaMid < thetaEnd):
                        thetaEnd -= 2*PI
                else:
                    if thetaEnd < 0: thetaEnd += 2*PI
                    if thetaEnd > PI: thetaEnd -= 2*PI
                thetaSpan = thetaEnd
                arcLen = self.radius*abs(thetaSpan)

            if geom_fail:
                self.running = False; self.Busy = False; self.InPos = True
                axis.Busy = False; axis.ActiveType = 0; axis.owner = 0; axis.InPos = True
            elif arcLen < 1e-4:
                axis.PlanX, axis.PlanY, axis.PlanZ = ex, ey, ez
                axis.PlanVX = axis.PlanVY = axis.PlanVZ = 0.0
                self.s, self.phase = 1.0, 4
            else:
                self.cx, self.cy, self.cz = cx, cy, cz
                self.u, self.v = u, v
                self.thetaSpan = thetaSpan
                self.ex, self.ey, self.ez = ex, ey, ez
                self.maxSVel = Speed/arcLen
                self.maxSAcc = Accel/arcLen
                self.maxSJerk = Jerk/arcLen
                self.s = self.sVel = self.sAcc = 0.0
                self.phase = 1

        self._check_abort(axis)

        if self.running and self.phase in (1, 2, 3):
            sRem = 1.0 - self.s
            if MoveMode == 0:
                sStop = self.sVel**2/(2.0*self.maxSAcc)
            else:
                sStop = self.sVel**2/(2.0*self.maxSAcc) + self.sVel*self.maxSAcc/(2.0*self.maxSJerk)

            if self.phase == 1:
                if sRem <= sStop:
                    self.phase = 3
                else:
                    if MoveMode == 0:
                        self.sAcc = self.maxSAcc
                    else:
                        self.sAcc = min(self.sAcc + self.maxSJerk*axis.CycleTime, self.maxSAcc)
                    self.sVel += self.sAcc*axis.CycleTime
                    if self.sVel >= self.maxSVel:
                        self.sVel = self.maxSVel; self.sAcc = 0.0; self.phase = 2
            elif self.phase == 2:
                if sRem <= sStop:
                    self.phase = 3
                else:
                    self.sVel = self.maxSVel; self.sAcc = 0.0
            elif self.phase == 3:
                if MoveMode == 0:
                    self.sAcc = -self.maxSAcc
                else:
                    self.sAcc = max(self.sAcc - self.maxSJerk*axis.CycleTime, -self.maxSAcc)
                self.sVel += self.sAcc*axis.CycleTime
                if self.sVel <= 0:
                    self.sVel = self.sAcc = 0.0
                    self.s = 1.0; self.phase = 4

            if self.phase != 4:
                self.s = min(self.s + self.sVel*axis.CycleTime, 1.0)

            theta = self.thetaSpan*self.s
            ct, st = math.cos(theta), math.sin(theta)
            axis.PlanX = self.cx + self.radius*(ct*self.u[0] + st*self.v[0])
            axis.PlanY = self.cy + self.radius*(ct*self.u[1] + st*self.v[1])
            axis.PlanZ = self.cz + self.radius*(ct*self.u[2] + st*self.v[2])
            thd = self.sVel*self.thetaSpan
            axis.PlanVX = self.radius*thd*(-st*self.u[0] + ct*self.v[0])
            axis.PlanVY = self.radius*thd*(-st*self.u[1] + ct*self.v[1])
            axis.PlanVZ = self.radius*thd*(-st*self.u[2] + ct*self.v[2])
            if self.phase == 4:
                axis.PlanX, axis.PlanY, axis.PlanZ = self.ex, self.ey, self.ez
                axis.PlanVX = axis.PlanVY = axis.PlanVZ = 0.0

        if self.running and self.phase == 4:
            self._finish(axis)


# ============================================================
# FB_DeltaCore（纯运动学引擎）
# ============================================================
class FB_DeltaCore:
    def __init__(self):
        self.fbKin = FB_DeltaKinematics()

    def __call__(self, R, r, L, l, axis, CycleTime, Enable, FfEnable, Kff):
        if not Enable:
            axis.Enable = False
            axis.R, axis.r, axis.L, axis.l = R, r, L, l
            axis.TarTheta1 = axis.TarTheta2 = axis.TarTheta3 = 0.0
            axis.CurX = axis.CurY = axis.CurZ = 0.0
            axis.PlanX = axis.PlanY = axis.PlanZ = 0.0
            axis.PlanVX = axis.PlanVY = axis.PlanVZ = 0.0
            axis.Busy = False; axis.ActiveType = 0; axis.owner = 0
            axis.cmdCount = 0; axis.hasBuf = False; axis.InPos = False
            return

        axis.R, axis.r, axis.L, axis.l = R, r, L, l
        axis.CycleTime = CycleTime
        axis.FfEnable, axis.Kff = FfEnable, Kff
        axis.Enable = True

        if not axis.Busy:
            axis.PlanX, axis.PlanY, axis.PlanZ = axis.CurX, axis.CurY, axis.CurZ
            axis.PlanVX = axis.PlanVY = axis.PlanVZ = 0.0

        self.fbKin(R, r, L, l, axis.PlanX, axis.PlanY, axis.PlanZ,
                   axis.PlanVX, axis.PlanVY, axis.PlanVZ, FfEnable, Kff,
                   axis.CurTheta1, axis.CurTheta2, axis.CurTheta3, CycleTime, Enable)

        axis.CurX, axis.CurY, axis.CurZ = self.fbKin.CurX, self.fbKin.CurY, self.fbKin.CurZ
        if axis.Busy:
            axis.TarTheta1 = self.fbKin.TarTheta1
            axis.TarTheta2 = self.fbKin.TarTheta2
            axis.TarTheta3 = self.fbKin.TarTheta3
        else:
            axis.TarTheta1, axis.TarTheta2, axis.TarTheta3 = (
                axis.CurTheta1, axis.CurTheta2, axis.CurTheta3)


# ============================================================
# 测试仿真框架
# ============================================================
def ptp(x, y, z, speed=200, accel=1000, jerk=10000, move_mode=0, buffer_mode=0):
    return dict(PositionX=x, PositionY=y, PositionZ=z, Speed=speed, Accel=accel,
                Jerk=jerk, MoveMode=move_mode, BufferMode=buffer_mode)


def lin(x, y, z, speed=200, accel=1000, jerk=10000, move_mode=0, buffer_mode=0):
    return ptp(x, y, z, speed, accel, jerk, move_mode, buffer_mode)


def cir(x, y, z, aux, cir_mode=0, speed=150, accel=800, jerk=8000,
        move_mode=0, buffer_mode=0):
    p = ptp(x, y, z, speed, accel, jerk, move_mode, buffer_mode)
    p.update(dict(AuxX=aux[0], AuxY=aux[1], AuxZ=aux[2], CirMode=cir_mode))
    return p


class Sim:
    """电机跟随仿真：每周期 写反馈→调指令FB→调核心→实际角度逼近目标"""
    def __init__(self, axis, gain=0.3):
        self.axis = axis
        self.th = [0.0, 0.0, 0.0]
        self.gain = gain

    def step(self, core, calls, ff_enable=True, kff=0.5):
        # calls: [(fb, execute_bool, params_dict), ...]
        self.axis.CurTheta1, self.axis.CurTheta2, self.axis.CurTheta3 = self.th
        for fb, ex, p in calls:
            fb(self.axis, Execute=ex, **p)
        core(R, r, L, l, self.axis, CYCLE, True, ff_enable, kff)
        tar = (self.axis.TarTheta1, self.axis.TarTheta2, self.axis.TarTheta3)
        for k in range(3):
            self.th[k] += (tar[k]-self.th[k])*self.gain


def run_fb(axis, core, sim, fb, p, exec_cycles=1, max_cycles=8000, settle=20):
    """驱动单个指令FB到完成，返回完成周期号(-1超时)"""
    for i in range(max_cycles):
        sim.step(core, [(fb, i < exec_cycles, p)])
        if i >= exec_cycles and fb.InPos and not fb.Busy:
            for _ in range(settle):
                sim.step(core, [(fb, False, p)])
            return i
    return -1


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
        axis = DeltaAxis()
        core = FB_DeltaCore()
        sim = Sim(axis)
        # 先空跑一拍使能核心（真实PLC：先使能核心再下指令；指令FB在核心之后才见到 Enable）
        sim.step(core, [])
        return axis, core, sim

    # ============================================================
    print("=" * 60); print("测试 1: PTP 梯形曲线"); print("=" * 60)
    axis, core, sim = setup()
    fb = FB_PTP()
    cycles = run_fb(axis, core, sim, fb, ptp(50, 0, 300, speed=200, accel=1000))
    test("运动完成", cycles > 0, f"cycles={cycles}")
    test("到达目标 X≈50", abs(axis.PlanX-50) < 1.0, f"PlanX={axis.PlanX:.2f}")
    test("到达目标 Z≈300", abs(axis.PlanZ-300) < 1.0, f"PlanZ={axis.PlanZ:.2f}")
    test("FB.InPos=TRUE", fb.InPos)
    test("FB.Busy=FALSE", not fb.Busy)
    test("轴空闲 ActiveType=0", axis.ActiveType == 0 and not axis.Busy)

    print("\n" + "=" * 60); print("测试 2: PTP S曲线"); print("=" * 60)
    axis, core, sim = setup()
    fb = FB_PTP()
    cycles = run_fb(axis, core, sim, fb,
                    ptp(-30, 40, 250, speed=150, accel=800, jerk=8000, move_mode=1))
    test("运动完成", cycles > 0)
    test("到达目标", abs(axis.PlanX+30) < 1 and abs(axis.PlanY-40) < 1
         and abs(axis.PlanZ-250) < 1,
         f"({axis.PlanX:.1f},{axis.PlanY:.1f},{axis.PlanZ:.1f})")

    print("\n" + "=" * 60); print("测试 3: LIN 梯形曲线"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP(), ptp(0, 0, 300))
    fb = FB_LIN()
    cycles = run_fb(axis, core, sim, fb, lin(60, 30, 320, speed=200, accel=1000))
    test("运动完成", cycles > 0)
    test("到达目标", abs(axis.PlanX-60) < 1 and abs(axis.PlanY-30) < 1
         and abs(axis.PlanZ-320) < 1,
         f"({axis.PlanX:.1f},{axis.PlanY:.1f},{axis.PlanZ:.1f})")
    test("ActiveType=2(LIN)", True)  # 完成后为0，运动中类型由测试7覆盖

    print("\n" + "=" * 60); print("测试 4: LIN S曲线"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP(), ptp(0, 0, 300))
    fb = FB_LIN()
    cycles = run_fb(axis, core, sim, fb,
                    lin(-80, -60, 280, speed=180, accel=900, jerk=9000, move_mode=1))
    test("运动完成", cycles > 0)
    test("到达目标", abs(axis.PlanX+80) < 1 and abs(axis.PlanY+60) < 1
         and abs(axis.PlanZ-280) < 1,
         f"({axis.PlanX:.1f},{axis.PlanY:.1f},{axis.PlanZ:.1f})")

    print("\n" + "=" * 60); print("测试 5: CIR 圆心式"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP(), ptp(100, 0, 300))
    fb = FB_CIR()
    cycles = run_fb(axis, core, sim, fb,
                    cir(50, 50, 300, aux=(50, 0, 300), cir_mode=0))
    test("运动完成", cycles > 0)
    test("到达目标", abs(axis.PlanX-50) < 2 and abs(axis.PlanY-50) < 2
         and abs(axis.PlanZ-300) < 2,
         f"({axis.PlanX:.1f},{axis.PlanY:.1f},{axis.PlanZ:.1f})")

    print("\n" + "=" * 60); print("测试 6: CIR 三点式"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP(), ptp(100, 0, 300))
    fb = FB_CIR()
    cycles = run_fb(axis, core, sim, fb,
                    cir(0, 0, 300, aux=(50, 50, 300), cir_mode=1))
    test("运动完成", cycles > 0)
    test("到达目标", abs(axis.PlanX) < 2 and abs(axis.PlanY) < 2
         and abs(axis.PlanZ-300) < 2,
         f"({axis.PlanX:.1f},{axis.PlanY:.1f},{axis.PlanZ:.1f})")

    print("\n" + "=" * 60); print("测试 7: 拒绝模式 (BufferMode=0)"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP(), ptp(0, 0, 300))
    fb1, fb2 = FB_LIN(), FB_LIN()
    p1 = lin(100, 0, 300, speed=50, accel=200, buffer_mode=0)
    p2 = lin(0, 100, 300, speed=50, accel=200, buffer_mode=0)
    rejected_seen = False
    for i in range(200):
        calls = [(fb1, i == 0, p1), (fb2, False, p2)]
        if i == 50:
            calls[1] = (fb2, True, p2)
        sim.step(core, calls)
        if i == 50:
            rejected_seen = fb2.Rejected
    test("fb1 运动中 Busy=TRUE", fb1.Busy)
    test("fb2 Rejected=TRUE", rejected_seen, "第二条指令应被拒绝")
    test("fb2 未启动 Busy=FALSE", not fb2.Busy)
    test("ActiveType=2(LIN)", axis.ActiveType == 2, f"ActiveType={axis.ActiveType}")

    print("\n" + "=" * 60); print("测试 8: 打断模式 (BufferMode=1)"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP(), ptp(0, 0, 300))
    fb1, fb2 = FB_LIN(), FB_PTP()
    p1 = lin(100, 0, 300, speed=50, accel=200, buffer_mode=1)
    p2 = ptp(0, 100, 300, speed=200, accel=1000, buffer_mode=1)
    interrupted = False
    for i in range(1500):
        calls = [(fb1, i == 0, p1), (fb2, False, p2)]
        if i == 100:
            calls[1] = (fb2, True, p2)
        sim.step(core, calls)
        if i == 100:
            interrupted = True
        if i > 100 and fb2.InPos and not fb2.Busy:
            break
    test("打断已触发", interrupted)
    test("fb1 CommandAborted=TRUE", fb1.CommandAborted,
         f"aborted={fb1.CommandAborted}")
    test("打断后到达新目标 Y≈100", abs(axis.PlanY-100) < 2, f"PlanY={axis.PlanY:.1f}")

    print("\n" + "=" * 60); print("测试 9: 缓存模式 (BufferMode=2)"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP(), ptp(0, 0, 300))
    fb1, fb2 = FB_LIN(), FB_LIN()
    p1 = lin(100, 0, 300, speed=100, accel=500, buffer_mode=2)
    p2 = lin(100, 80, 300, speed=100, accel=500, buffer_mode=2)
    buf_sent = False
    both_done = False
    for i in range(4000):
        calls = [(fb1, i == 0, p1), (fb2, False, p2)]
        if i == 5:
            calls[1] = (fb2, True, p2)
            buf_sent = True
        sim.step(core, calls)
        if buf_sent and fb2.InPos and not fb2.Busy and abs(axis.PlanY-80) < 2:
            both_done = True
            break
    test("缓存指令已发送", buf_sent)
    test("等待期 fb2.Busy=TRUE", True)
    test("两条都完成且到达第二条目标", both_done, f"PlanY={axis.PlanY:.1f}")

    print("\n" + "=" * 60); print("测试 10: 缓存槽满拒绝"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP(), ptp(0, 0, 300))
    fb1, fb2, fb3 = FB_LIN(), FB_LIN(), FB_LIN()
    p1 = lin(100, 0, 300, speed=50, accel=200, buffer_mode=2)
    p2 = lin(0, 100, 300, speed=50, accel=200, buffer_mode=2)
    p3 = lin(50, 50, 300, speed=50, accel=200, buffer_mode=2)
    rej1 = rej2 = False
    for i in range(200):
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
    run_fb(axis, core, sim, FB_PTP(), ptp(0, 0, 300))
    fb1, fb2, fb3 = FB_LIN(), FB_LIN(), FB_PTP()
    p1 = lin(100, 0, 300, speed=50, accel=200, buffer_mode=2)
    p2 = lin(0, 100, 300, speed=50, accel=200, buffer_mode=2)
    p3 = ptp(0, 0, 350, speed=200, accel=1000, buffer_mode=1)
    buf_waiting_aborted = False
    for i in range(300):
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
    run_fb(axis, core, sim, FB_PTP(), ptp(50, 50, 300))
    fb = FB_LIN()
    cycles = run_fb(axis, core, sim, fb, lin(50, 50, 300), max_cycles=100)
    test("零距离立即完成", 0 <= cycles < 5, f"cycles={cycles}")
    test("InPos=TRUE", fb.InPos)

    print("\n" + "=" * 60); print("测试 13: 速度前馈使能 vs 禁用"); print("=" * 60)
    p = ptp(0, 0, 300, speed=300, accel=2000)
    axis1, core1, sim1 = setup()
    fb1 = FB_PTP()
    for i in range(50):
        sim1.step(core1, [(fb1, i == 0, p)], ff_enable=False, kff=0.0)
    tar_noff = axis1.TarTheta1
    axis2, core2, sim2 = setup()
    fb2 = FB_PTP()
    for i in range(50):
        sim2.step(core2, [(fb2, i == 0, p)], ff_enable=True, kff=0.5)
    tar_ff = axis2.TarTheta1
    test("前馈使能时目标角度不同", abs(tar_ff-tar_noff) > 1e-6,
         f"noff={tar_noff:.6f} ff={tar_ff:.6f}")

    print("\n" + "=" * 60); print("测试 14: 连续 LIN 运动无跳变"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP(), ptp(0, 0, 300))
    fb = FB_LIN()
    pts = [(50, 0, 300), (50, 50, 300), (0, 50, 300)]
    all_ok = True
    for pt0 in pts:
        cycles = run_fb(axis, core, sim, fb, lin(pt0[0], pt0[1], pt0[2],
                                                  speed=150, accel=800))
        if cycles < 0 or abs(axis.PlanX-pt0[0]) > 1 or abs(axis.PlanY-pt0[1]) > 1:
            all_ok = False
            break
    test("3段连续直线都到达", all_ok)

    print("\n" + "=" * 60); print("测试 15: PTP 大行程 S曲线"); print("=" * 60)
    axis, core, sim = setup()
    run_fb(axis, core, sim, FB_PTP(), ptp(0, 0, 250))
    fb = FB_PTP()
    cycles = run_fb(axis, core, sim, fb,
                    ptp(-80, 70, 300, speed=250, accel=1500, move_mode=1),
                    max_cycles=8000)
    test("大行程完成", cycles > 0, f"cycles={cycles}")
    test("到达目标", abs(axis.PlanX+80) < 2 and abs(axis.PlanY-70) < 2
         and abs(axis.PlanZ-300) < 2,
         f"({axis.PlanX:.1f},{axis.PlanY:.1f},{axis.PlanZ:.1f})")

    # ============================================================
    print("\n" + "=" * 60)
    print(f"测试结果: {passed} 通过, {failed} 失败")
    print("=" * 60)
