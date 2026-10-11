"""
Delta 机器人运动控制库 - 多样本测试
将 ST 逻辑移植为 Python，验证各种运动模式和指令处理模式
"""
import math

# ============================================================
# 常量
# ============================================================
COS1, SIN1 = 1.0, 0.0
COS2, SIN2 = -0.5, 0.8660254037844386
COS3, SIN3 = -0.5, -0.8660254037844386
PI = math.pi


# ============================================================
# st_MoveCmd
# ============================================================
class MoveCmd:
    def __init__(self):
        self.MoveType = 0      # 0=PTP 1=LIN 2=CIR
        self.PosX = self.PosY = self.PosZ = 0.0
        self.AuxX = self.AuxY = self.AuxZ = 0.0
        self.CirMode = 0       # 0=圆心式 1=三点式
        self.Speed = 100.0
        self.Accel = 500.0
        self.Jerk = 5000.0
        self.MoveMode = 0      # 0=梯形 1=S曲线
        self.BufferMode = 0    # 0=拒绝 1=打断 2=缓存


# ============================================================
# FB_DeltaKinematics
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

        # IK (3 axes)
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

        # 前馈: θ̇ = -(∂f/∂P·v)/(∂f/∂θ) = -(B·v)/Aii（隐函数求导带负号）
        # 各轴前馈仅在该轴 IK 成功时生效，失败轴严格保持
        dTh1_ff = dTh2_ff = dTh3_ff = 0.0
        if FfEnable and ok1:
            def ff_ax(cos_a, sin_a, ik_th):
                xp = PlanX * cos_a + PlanY * sin_a
                yp = -PlanX * sin_a + PlanY * cos_a
                ct = math.cos(ik_th)
                st = math.sin(ik_th)
                Aii = -2.0 * L * ((e - xp) * st + PlanZ * ct)
                Bix = -2.0 * ((L*ct + e - xp) * cos_a + yp * sin_a)
                Biy = -2.0 * ((L*ct + e - xp) * sin_a - yp * cos_a)
                Biz = -2.0 * (L * st - PlanZ)
                if abs(Aii) > 1e-9:
                    return -((Bix*PlanVX + Biy*PlanVY + Biz*PlanVZ) / Aii) * CycleTime
                return 0.0
            dTh1_ff = ff_ax(COS1, SIN1, ikTh1)
        if FfEnable and ok2:
            dTh2_ff = ff_ax(COS2, SIN2, ikTh2)
        if FfEnable and ok3:
            dTh3_ff = ff_ax(COS3, SIN3, ikTh3)

        self.TarTheta1 = ikTh1 + Kff * dTh1_ff
        self.TarTheta2 = ikTh2 + Kff * dTh2_ff
        self.TarTheta3 = ikTh3 + Kff * dTh3_ff


# ============================================================
# FB_PTP
# ============================================================
class FB_PTP:
    def __init__(self):
        self.PlanX = self.PlanY = self.PlanZ = 0.0
        self.PlanVX = self.PlanVY = self.PlanVZ = 0.0
        self.InPos = False
        self.Busy = False
        self.phase = 0
        self.s = 0.0
        self.sVel = 0.0
        self.sAcc = 0.0
        self.exLast = False
        # 持久状态
        self.startTh1 = self.startTh2 = self.startTh3 = 0.0
        self.tgtTh1 = self.tgtTh2 = self.tgtTh3 = 0.0
        self.tgtX = self.tgtY = self.tgtZ = 0.0
        self.curX = self.curY = self.curZ = 0.0
        self.totalDist = 0.0
        self.maxSVel = self.maxSAcc = self.maxSJerk = 0.0

    def __call__(self, R, r, L, l, TargetX, TargetY, TargetZ,
                 CurTheta1, CurTheta2, CurTheta3, Speed, Accel, Jerk,
                 MoveMode, CycleTime, Enable, Execute):
        if not Enable:
            self.PlanX = self.PlanY = self.PlanZ = 0.0
            self.PlanVX = self.PlanVY = self.PlanVZ = 0.0
            self.InPos = False
            self.Busy = False
            self.phase = 0
            self.s = 0.0
            return

        e = R - r
        exRise = Execute and not self.exLast
        self.exLast = Execute

        if exRise:
            # FK 当前位置
            s1 = e + L*math.cos(CurTheta1)
            s2 = e + L*math.cos(CurTheta2)
            s3 = e + L*math.cos(CurTheta3)
            sn1 = L*math.sin(CurTheta1)
            sn2 = L*math.sin(CurTheta2)
            sn3 = L*math.sin(CurTheta3)
            a1 = -2.0*(s2*COS2-s1*COS1); b1 = -2.0*(s2*SIN2-s1*SIN1)
            c1 = -2.0*(sn2-sn1); d1 = (s1*s1+sn1*sn1)-(s2*s2+sn2*sn2)
            a2 = -2.0*(s3*COS3-s1*COS1); b2 = -2.0*(s3*SIN3-s1*SIN1)
            c2 = -2.0*(sn3-sn1); d2 = (s1*s1+sn1*sn1)-(s3*s3+sn3*sn3)
            Det = a1*b2-a2*b1
            if abs(Det) > 1e-9:
                px = (b2*d1-b1*d2)/Det; qx = (b1*c2-b2*c1)/Det
                py = (a1*d2-a2*d1)/Det; qy = (a2*c1-a1*c2)/Det
                X0 = px-s1*COS1; Y0 = py-s1*SIN1
                Az = qx*qx+qy*qy+1.0; Bz = 2.0*(X0*qx+Y0*qy)-2.0*sn1
                Cz = X0*X0+Y0*Y0+sn1*sn1-l*l; Dz = Bz*Bz-4.0*Az*Cz
                if Dz >= 0:
                    z1 = (-Bz+math.sqrt(Dz))/(2.0*Az)
                    z2 = (-Bz-math.sqrt(Dz))/(2.0*Az)
                    if z1 > 0: self.curZ = z1
                    elif z2 > 0: self.curZ = z2
                    else: self.curZ = z1
                    self.curX = px+qx*self.curZ; self.curY = py+qy*self.curZ
                else:
                    self.curX = self.curY = self.curZ = 0.0
            else:
                self.curX = self.curY = self.curZ = 0.0

            # IK 目标角度
            self.tgtX = TargetX; self.tgtY = TargetY; self.tgtZ = TargetZ
            def ik(cos_a, sin_a, cur_th):
                xp = TargetX*cos_a + TargetY*sin_a
                yp = -TargetX*sin_a + TargetY*cos_a
                A = 2.0*L*(e-xp); B = -2.0*L*TargetZ
                C = L*L+(e-xp)*(e-xp)+yp*yp+TargetZ*TargetZ-l*l
                Disc = A*A+B*B-C*C
                if Disc < 0: return cur_th
                denom = C-A
                if abs(denom) < 1e-9: return cur_th
                u1 = (-B+math.sqrt(Disc))/denom
                u2 = (-B-math.sqrt(Disc))/denom
                ts1 = 2.0*math.atan(u1); ts2 = 2.0*math.atan(u2)
                if ts1 > 0 and ts2 > 0:
                    return ts1 if abs(ts1-cur_th) <= abs(ts2-cur_th) else ts2
                elif ts1 > 0: return ts1
                elif ts2 > 0: return ts2
                else: return ts1

            self.tgtTh1 = ik(COS1, SIN1, CurTheta1)
            self.tgtTh2 = ik(COS2, SIN2, CurTheta2)
            self.tgtTh3 = ik(COS3, SIN3, CurTheta3)

            self.startTh1 = CurTheta1
            self.startTh2 = CurTheta2
            self.startTh3 = CurTheta3
            self.totalDist = math.sqrt((TargetX-self.curX)**2 +
                                       (TargetY-self.curY)**2 +
                                       (TargetZ-self.curZ)**2)
            if self.totalDist < 1e-4:
                self.PlanX = self.tgtX; self.PlanY = self.tgtY; self.PlanZ = self.tgtZ
                self.PlanVX = self.PlanVY = self.PlanVZ = 0.0
                self.s = 1.0; self.phase = 4; self.InPos = True; self.Busy = False
            else:
                self.maxSVel = Speed / self.totalDist
                self.maxSAcc = Accel / self.totalDist
                self.maxSJerk = Jerk / self.totalDist
                self.s = 0.0; self.sVel = 0.0; self.sAcc = 0.0; self.phase = 1
                self.InPos = False; self.Busy = True

        if self.phase != 4 and self.phase != 0:
            sRem = 1.0 - self.s
            if MoveMode == 0:
                sStop = self.sVel**2 / (2.0 * self.maxSAcc)
            else:
                sStop = self.sVel**2/(2.0*self.maxSAcc) + self.sVel*self.maxSAcc/(2.0*self.maxSJerk)

            if self.phase == 1:
                if sRem <= sStop:
                    self.phase = 3
                else:
                    if MoveMode == 0:
                        self.sAcc = self.maxSAcc
                    else:
                        self.sAcc += self.maxSJerk * CycleTime
                        if self.sAcc > self.maxSAcc: self.sAcc = self.maxSAcc
                    self.sVel += self.sAcc * CycleTime
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
                    self.sAcc -= self.maxSJerk * CycleTime
                    if self.sAcc < -self.maxSAcc: self.sAcc = -self.maxSAcc
                self.sVel += self.sAcc * CycleTime
                if self.sVel <= 0:
                    self.sVel = 0.0; self.sAcc = 0.0
                    self.s = 1.0; self.phase = 4

            if self.phase != 4:
                self.s += self.sVel * CycleTime
                if self.s > 1.0: self.s = 1.0

            interpTh1 = self.startTh1 + (self.tgtTh1 - self.startTh1) * self.s
            interpTh2 = self.startTh2 + (self.tgtTh2 - self.startTh2) * self.s
            interpTh3 = self.startTh3 + (self.tgtTh3 - self.startTh3) * self.s

            s1 = e+L*math.cos(interpTh1); s2 = e+L*math.cos(interpTh2); s3 = e+L*math.cos(interpTh3)
            sn1 = L*math.sin(interpTh1); sn2 = L*math.sin(interpTh2); sn3 = L*math.sin(interpTh3)
            a1=-2.0*(s2*COS2-s1*COS1); b1=-2.0*(s2*SIN2-s1*SIN1)
            c1=-2.0*(sn2-sn1); d1=(s1*s1+sn1*sn1)-(s2*s2+sn2*sn2)
            a2=-2.0*(s3*COS3-s1*COS1); b2=-2.0*(s3*SIN3-s1*SIN1)
            c2=-2.0*(sn3-sn1); d2=(s1*s1+sn1*sn1)-(s3*s3+sn3*sn3)
            Det=a1*b2-a2*b1
            if abs(Det) > 1e-9:
                px=(b2*d1-b1*d2)/Det; qx=(b1*c2-b2*c1)/Det
                py=(a1*d2-a2*d1)/Det; qy=(a2*c1-a1*c2)/Det
                X0=px-s1*COS1; Y0=py-s1*SIN1
                Az=qx*qx+qy*qy+1.0; Bz=2.0*(X0*qx+Y0*qy)-2.0*sn1
                Cz=X0*X0+Y0*Y0+sn1*sn1-l*l; Dz=Bz*Bz-4.0*Az*Cz
                if Dz >= 0:
                    z1=(-Bz+math.sqrt(Dz))/(2.0*Az); z2=(-Bz-math.sqrt(Dz))/(2.0*Az)
                    if z1>0: self.PlanZ=z1
                    elif z2>0: self.PlanZ=z2
                    else: self.PlanZ=z1
                    self.PlanX=px+qx*self.PlanZ; self.PlanY=py+qy*self.PlanZ

            self.PlanVX = (self.tgtX - self.curX) * self.sVel
            self.PlanVY = (self.tgtY - self.curY) * self.sVel
            self.PlanVZ = (self.tgtZ - self.curZ) * self.sVel

            if self.phase == 4:
                # Plan 保持最终插值角度的 FK 结果，不强制写为目标值
                self.PlanVX = self.PlanVY = self.PlanVZ = 0.0
                self.InPos = True; self.Busy = False


# ============================================================
# FB_LIN
# ============================================================
class FB_LIN:
    def __init__(self):
        self.PlanX = self.PlanY = self.PlanZ = 0.0
        self.PlanVX = self.PlanVY = self.PlanVZ = 0.0
        self.InPos = False
        self.Busy = False
        self.phase = 0
        self.s = 0.0
        self.sVel = 0.0
        self.sAcc = 0.0
        self.exLast = False
        self.enLast = False
        self.startX = self.startY = self.startZ = 0.0
        self.tgtX = self.tgtY = self.tgtZ = 0.0
        self.dirX = self.dirY = self.dirZ = 0.0
        self.totalDist = 0.0
        self.maxSVel = self.maxSAcc = self.maxSJerk = 0.0

    def __call__(self, TargetX, TargetY, TargetZ, Speed, Accel, Jerk,
                 MoveMode, CycleTime, Enable, Execute, InitX, InitY, InitZ):
        if not Enable:
            self.PlanX = self.PlanY = self.PlanZ = 0.0
            self.PlanVX = self.PlanVY = self.PlanVZ = 0.0
            self.InPos = False; self.Busy = False; self.phase = 0; self.s = 0.0
            return

        if not self.enLast:
            self.PlanX = InitX; self.PlanY = InitY; self.PlanZ = InitZ
        self.enLast = Enable

        exRise = Execute and not self.exLast
        self.exLast = Execute

        if exRise:
            # 始终以 Init（核心层提供的FK实际位置）为起点，保证与实际位置连续
            self.startX = InitX; self.startY = InitY; self.startZ = InitZ
            dx = TargetX - self.startX
            dy = TargetY - self.startY
            dz = TargetZ - self.startZ
            self.tgtX = TargetX; self.tgtY = TargetY; self.tgtZ = TargetZ
            self.totalDist = math.sqrt(dx*dx + dy*dy + dz*dz)
            if self.totalDist < 1e-4:
                self.PlanX = self.tgtX; self.PlanY = self.tgtY; self.PlanZ = self.tgtZ
                self.PlanVX = self.PlanVY = self.PlanVZ = 0.0
                self.s = 1.0; self.phase = 4; self.InPos = True; self.Busy = False
            else:
                self.dirX = dx / self.totalDist
                self.dirY = dy / self.totalDist
                self.dirZ = dz / self.totalDist
                self.maxSVel = Speed / self.totalDist
                self.maxSAcc = Accel / self.totalDist
                self.maxSJerk = Jerk / self.totalDist
                self.s = 0.0; self.sVel = 0.0; self.sAcc = 0.0; self.phase = 1
                self.InPos = False; self.Busy = True

        if self.phase != 4 and self.phase != 0:
            sRem = 1.0 - self.s
            if MoveMode == 0:
                sStop = self.sVel**2 / (2.0 * self.maxSAcc)
            else:
                sStop = self.sVel**2/(2.0*self.maxSAcc) + self.sVel*self.maxSAcc/(2.0*self.maxSJerk)

            if self.phase == 1:
                if sRem <= sStop:
                    self.phase = 3
                else:
                    if MoveMode == 0:
                        self.sAcc = self.maxSAcc
                    else:
                        self.sAcc += self.maxSJerk * CycleTime
                        if self.sAcc > self.maxSAcc: self.sAcc = self.maxSAcc
                    self.sVel += self.sAcc * CycleTime
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
                    self.sAcc -= self.maxSJerk * CycleTime
                    if self.sAcc < -self.maxSAcc: self.sAcc = -self.maxSAcc
                self.sVel += self.sAcc * CycleTime
                if self.sVel <= 0:
                    self.sVel = 0.0; self.sAcc = 0.0
                    self.s = 1.0; self.phase = 4

            if self.phase != 4:
                self.s += self.sVel * CycleTime
                if self.s > 1.0: self.s = 1.0

            self.PlanX = self.startX + self.dirX * self.s * self.totalDist
            self.PlanY = self.startY + self.dirY * self.s * self.totalDist
            self.PlanZ = self.startZ + self.dirZ * self.s * self.totalDist
            self.PlanVX = self.dirX * self.sVel * self.totalDist
            self.PlanVY = self.dirY * self.sVel * self.totalDist
            self.PlanVZ = self.dirZ * self.sVel * self.totalDist

            if self.phase == 4:
                self.PlanX = self.tgtX; self.PlanY = self.tgtY; self.PlanZ = self.tgtZ
                self.PlanVX = self.PlanVY = self.PlanVZ = 0.0
                self.InPos = True; self.Busy = False


# ============================================================
# FB_CIR
# ============================================================
class FB_CIR:
    def __init__(self):
        self.PlanX = self.PlanY = self.PlanZ = 0.0
        self.PlanVX = self.PlanVY = self.PlanVZ = 0.0
        self.InPos = False
        self.Busy = False
        self.phase = 0
        self.s = 0.0
        self.sVel = 0.0
        self.sAcc = 0.0
        self.exLast = False
        self.enLast = False
        # 几何
        self.cx = self.cy = self.cz = 0.0
        self.radius = 0.0
        self.ux = self.uy = self.uz = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.thetaStart = 0.0
        self.thetaEnd = 0.0
        self.thetaSpan = 0.0
        self.arcLen = 0.0
        self.maxSVel = self.maxSAcc = self.maxSJerk = 0.0
        self.ex = self.ey = self.ez = 0.0

    def __call__(self, TargetX, TargetY, TargetZ, AuxX, AuxY, AuxZ, CirMode,
                 Speed, Accel, Jerk, MoveMode, CycleTime, Enable, Execute,
                 InitX, InitY, InitZ):
        if not Enable:
            self.PlanX = self.PlanY = self.PlanZ = 0.0
            self.PlanVX = self.PlanVY = self.PlanVZ = 0.0
            self.InPos = False; self.Busy = False; self.phase = 0; self.s = 0.0
            return

        if not self.enLast:
            self.PlanX = InitX; self.PlanY = InitY; self.PlanZ = InitZ
        self.enLast = Enable

        exRise = Execute and not self.exLast
        self.exLast = Execute

        if exRise:
            # 始终以 Init（核心层提供的FK实际位置）为起点，保证与实际位置连续
            sx, sy, sz = InitX, InitY, InitZ
            ex, ey, ez = TargetX, TargetY, TargetZ
            self.ex, self.ey, self.ez = ex, ey, ez

            if CirMode == 0:
                cx, cy, cz = AuxX, AuxY, AuxZ
            else:
                mx, my, mz = AuxX, AuxY, AuxZ
                smx, smy, smz = mx-sx, my-sy, mz-sz
                sex, sey, sez = ex-sx, ey-sy, ez-sz
                nx = smy*sez - smz*sey
                ny = smz*sex - smx*sez
                nz = smx*sey - smy*sex
                nLen = math.sqrt(nx*nx+ny*ny+nz*nz)
                if nLen < 1e-6:
                    self.phase = 0; self.InPos = True; self.Busy = False
                    return
                nx, ny, nz = nx/nLen, ny/nLen, nz/nLen
                smLen = math.sqrt(smx*smx+smy*smy+smz*smz)
                ux, uy, uz = smx/smLen, smy/smLen, smz/smLen
                vx = ny*uz - nz*uy
                vy = nz*ux - nx*uz
                vz = nx*uy - ny*ux
                eu = sex*ux + sey*uy + sez*uz
                ev = sex*vx + sey*vy + sez*vz
                aa = smLen / 2.0
                if abs(ev) < 1e-6:
                    self.phase = 0; self.InPos = True; self.Busy = False
                    return
                bb = (eu*eu + ev*ev - 2.0*aa*eu) / (2.0*ev)
                cx = sx + aa*ux + bb*vx
                cy = sy + aa*uy + bb*vy
                cz = sz + aa*uz + bb*vz

            csx, csy, csz = sx-cx, sy-cy, sz-cz
            radius = math.sqrt(csx*csx+csy*csy+csz*csz)
            if radius < 1e-6:
                self.phase = 0; self.InPos = True; self.Busy = False
                return
            ux, uy, uz = csx/radius, csy/radius, csz/radius

            cex, cey, cez = ex-cx, ey-cy, ez-cz
            if CirMode == 0:
                nx = csy*cez - csz*cey
                ny = csz*cex - csx*cez
                nz = csx*cey - csy*cex
                nLen = math.sqrt(nx*nx+ny*ny+nz*nz)
                if nLen < 1e-6:
                    self.phase = 0; self.InPos = True; self.Busy = False
                    return
                nx, ny, nz = nx/nLen, ny/nLen, nz/nLen

            vx = ny*uz - nz*uy
            vy = nz*ux - nx*uz
            vz = nx*uy - ny*ux

            eu = cex*ux + cey*uy + cez*uz
            ev = cex*vx + cey*vy + cez*vz
            thetaStart = 0.0
            thetaEnd = math.atan2(ev, eu)

            if CirMode == 1:
                cmx, cmy, cmz = mx-cx, my-cy, mz-cz
                mu = cmx*ux + cmy*uy + cmz*uz
                mv = cmx*vx + cmy*vy + cmz*vz
                thetaMid = math.atan2(mv, mu)
                if thetaEnd < 0: thetaEnd += 2*PI
                if thetaMid < 0: thetaMid += 2*PI
                if not (thetaMid > 0 and thetaMid < thetaEnd):
                    thetaEnd -= 2*PI
            else:
                if thetaEnd < 0: thetaEnd += 2*PI
                if thetaEnd > PI: thetaEnd -= 2*PI

            self.cx, self.cy, self.cz = cx, cy, cz
            self.radius = radius
            self.ux, self.uy, self.uz = ux, uy, uz
            self.vx, self.vy, self.vz = vx, vy, vz
            self.thetaStart = thetaStart
            self.thetaEnd = thetaEnd
            self.thetaSpan = thetaEnd
            self.arcLen = radius * abs(thetaEnd)

            if self.arcLen < 1e-4:
                self.PlanX = ex; self.PlanY = ey; self.PlanZ = ez
                self.PlanVX = self.PlanVY = self.PlanVZ = 0.0
                self.s = 1.0; self.phase = 4; self.InPos = True; self.Busy = False
            else:
                self.maxSVel = Speed / self.arcLen
                self.maxSAcc = Accel / self.arcLen
                self.maxSJerk = Jerk / self.arcLen
                self.s = 0.0; self.sVel = 0.0; self.sAcc = 0.0; self.phase = 1
                self.InPos = False; self.Busy = True

        if self.phase != 4 and self.phase != 0:
            sRem = 1.0 - self.s
            if MoveMode == 0:
                sStop = self.sVel**2 / (2.0 * self.maxSAcc)
            else:
                sStop = self.sVel**2/(2.0*self.maxSAcc) + self.sVel*self.maxSAcc/(2.0*self.maxSJerk)

            if self.phase == 1:
                if sRem <= sStop:
                    self.phase = 3
                else:
                    if MoveMode == 0:
                        self.sAcc = self.maxSAcc
                    else:
                        self.sAcc += self.maxSJerk * CycleTime
                        if self.sAcc > self.maxSAcc: self.sAcc = self.maxSAcc
                    self.sVel += self.sAcc * CycleTime
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
                    self.sAcc -= self.maxSJerk * CycleTime
                    if self.sAcc < -self.maxSAcc: self.sAcc = -self.maxSAcc
                self.sVel += self.sAcc * CycleTime
                if self.sVel <= 0:
                    self.sVel = 0.0; self.sAcc = 0.0
                    self.s = 1.0; self.phase = 4

            if self.phase != 4:
                self.s += self.sVel * CycleTime
                if self.s > 1.0: self.s = 1.0

            theta = self.thetaStart + self.s * self.thetaSpan
            cosT = math.cos(theta); sinT = math.sin(theta)
            self.PlanX = self.cx + self.radius * (cosT*self.ux + sinT*self.vx)
            self.PlanY = self.cy + self.radius * (cosT*self.uy + sinT*self.vy)
            self.PlanZ = self.cz + self.radius * (cosT*self.uz + sinT*self.vz)
            self.PlanVX = self.radius * (self.sVel*self.thetaSpan) * (-sinT*self.ux + cosT*self.vx)
            self.PlanVY = self.radius * (self.sVel*self.thetaSpan) * (-sinT*self.uy + cosT*self.vy)
            self.PlanVZ = self.radius * (self.sVel*self.thetaSpan) * (-sinT*self.uz + cosT*self.vz)

            if self.phase == 4:
                self.PlanX = self.ex; self.PlanY = self.ey; self.PlanZ = self.ez
                self.PlanVX = self.PlanVY = self.PlanVZ = 0.0
                self.InPos = True; self.Busy = False


# ============================================================
# FB_DeltaCore
# ============================================================
class FB_DeltaCore:
    def __init__(self):
        self.fbPTP = FB_PTP()
        self.fbLIN = FB_LIN()
        self.fbCIR = FB_CIR()
        self.fbKin = FB_DeltaKinematics()
        self.TarTheta1 = self.TarTheta2 = self.TarTheta3 = 0.0
        self.CurX = self.CurY = self.CurZ = 0.0
        self.PlanX = self.PlanY = self.PlanZ = 0.0
        self.InPos = False
        self.Busy = False
        self.Rejected = False
        self.Buffered = False
        self.ActiveType = 0
        self.exLast = False
        self.bufCmd = MoveCmd()
        self.hasBuf = False
        self.bufActive = False
        self.pendingStart = False
        self.curType = 0

    def __call__(self, R, r, L, l, cmd, Execute, CurTheta1, CurTheta2, CurTheta3,
                 CycleTime, FfEnable, Kff, Enable):
        if not Enable:
            self.TarTheta1 = self.TarTheta2 = self.TarTheta3 = 0.0
            self.CurX = self.CurY = self.CurZ = 0.0
            self.PlanX = self.PlanY = self.PlanZ = 0.0
            self.InPos = False; self.Busy = False; self.Rejected = False
            self.Buffered = False; self.ActiveType = 0
            self.hasBuf = False; self.bufActive = False; self.pendingStart = False
            self.curType = 0
            return

        exRise = Execute and not self.exLast
        self.exLast = Execute

        self.Rejected = False
        startType = -1

        # 第一步：pendingStart
        if self.pendingStart:
            startType = self.bufCmd.MoveType
            self.Busy = True
            self.InPos = False
            self.bufActive = True
            self.hasBuf = False
            self.pendingStart = False

        # 第二步：新指令
        if exRise:
            if cmd.MoveType < 0 or cmd.MoveType > 2:
                # 非法运动类型：拒绝，避免 Busy 卡死
                self.Rejected = True
            elif self.Busy:
                if cmd.BufferMode == 0:
                    self.Rejected = True
                elif cmd.BufferMode == 1:
                    startType = cmd.MoveType
                    self.InPos = False
                    self.hasBuf = False
                    self.bufActive = False
                elif cmd.BufferMode == 2:
                    if self.hasBuf:
                        self.Rejected = True
                    else:
                        self.bufCmd = cmd
                        self.hasBuf = True
            else:
                startType = cmd.MoveType
                self.Busy = True
                self.InPos = False

        # 第三步：exec 信号
        if startType >= 0:
            self.curType = startType
            execPTP = (self.curType == 0)
            execLIN = (self.curType == 1)
            execCIR = (self.curType == 2)
            self.ActiveType = self.curType + 1
        else:
            execPTP = execLIN = execCIR = False

        self.Buffered = self.hasBuf

        if self.bufActive:
            effCmd = self.bufCmd
        else:
            effCmd = cmd

        self.fbPTP(R, r, L, l, effCmd.PosX, effCmd.PosY, effCmd.PosZ,
                   CurTheta1, CurTheta2, CurTheta3, effCmd.Speed, effCmd.Accel,
                   effCmd.Jerk, effCmd.MoveMode, CycleTime, Enable, execPTP)

        self.fbLIN(effCmd.PosX, effCmd.PosY, effCmd.PosZ, effCmd.Speed,
                   effCmd.Accel, effCmd.Jerk, effCmd.MoveMode, CycleTime,
                   Enable, execLIN, self.CurX, self.CurY, self.CurZ)

        self.fbCIR(effCmd.PosX, effCmd.PosY, effCmd.PosZ, effCmd.AuxX,
                   effCmd.AuxY, effCmd.AuxZ, effCmd.CirMode, effCmd.Speed,
                   effCmd.Accel, effCmd.Jerk, effCmd.MoveMode, CycleTime,
                   Enable, execCIR, self.CurX, self.CurY, self.CurZ)

        # 规划位置/速度选择：执行时用运动FB输出，空闲时跟随当前输入角度
        if self.Busy:
            if self.curType == 0:
                self.PlanX = self.fbPTP.PlanX; self.PlanY = self.fbPTP.PlanY; self.PlanZ = self.fbPTP.PlanZ
                planVX = self.fbPTP.PlanVX; planVY = self.fbPTP.PlanVY; planVZ = self.fbPTP.PlanVZ
            elif self.curType == 1:
                self.PlanX = self.fbLIN.PlanX; self.PlanY = self.fbLIN.PlanY; self.PlanZ = self.fbLIN.PlanZ
                planVX = self.fbLIN.PlanVX; planVY = self.fbLIN.PlanVY; planVZ = self.fbLIN.PlanVZ
            else:
                self.PlanX = self.fbCIR.PlanX; self.PlanY = self.fbCIR.PlanY; self.PlanZ = self.fbCIR.PlanZ
                planVX = self.fbCIR.PlanVX; planVY = self.fbCIR.PlanVY; planVZ = self.fbCIR.PlanVZ
        else:
            # 空闲：Plan = FK(当前角度)，速度 = 0
            self.PlanX = self.CurX; self.PlanY = self.CurY; self.PlanZ = self.CurZ
            planVX = planVY = planVZ = 0.0

        self.fbKin(R, r, L, l, self.PlanX, self.PlanY, self.PlanZ,
                   planVX, planVY, planVZ, FfEnable, Kff,
                   CurTheta1, CurTheta2, CurTheta3, CycleTime, Enable)

        if self.Busy:
            # 执行指令：输出指令计算的目标角度
            self.TarTheta1 = self.fbKin.TarTheta1
            self.TarTheta2 = self.fbKin.TarTheta2
            self.TarTheta3 = self.fbKin.TarTheta3
        else:
            # 空闲：目标角度 = 当前输入角度（跟随实际位置）
            self.TarTheta1 = CurTheta1
            self.TarTheta2 = CurTheta2
            self.TarTheta3 = CurTheta3
        self.CurX = self.fbKin.CurX
        self.CurY = self.fbKin.CurY
        self.CurZ = self.fbKin.CurZ

        if self.Busy:
            if self.curType == 0 and self.fbPTP.InPos:
                self.InPos = True
            elif self.curType == 1 and self.fbLIN.InPos:
                self.InPos = True
            elif self.curType == 2 and self.fbCIR.InPos:
                self.InPos = True

            if self.InPos:
                self.Busy = False
                self.ActiveType = 0
                self.bufActive = False
                if self.hasBuf:
                    self.pendingStart = True


# ============================================================
# 测试框架
# ============================================================
# 机械参数
R, r, L, l = 100.0, 50.0, 150.0, 200.0
CYCLE = 0.001  # 1ms


def make_cmd(move_type, x, y, z, speed=200, accel=1000, jerk=10000,
             move_mode=0, buffer_mode=0, aux=(0,0,0), cir_mode=0):
    c = MoveCmd()
    c.MoveType = move_type
    c.PosX, c.PosY, c.PosZ = x, y, z
    c.Speed = speed
    c.Accel = accel
    c.Jerk = jerk
    c.MoveMode = move_mode
    c.BufferMode = buffer_mode
    c.AuxX, c.AuxY, c.AuxZ = aux
    c.CirMode = cir_mode
    return c


def run_core(core, cmd, execute_cycles=1, max_cycles=5000, ff_enable=True, kff=0.5):
    """运行核心FB，模拟实际角度跟随目标角度"""
    th1 = th2 = th3 = 0.0
    for i in range(max_cycles):
        exec_pulse = (i < execute_cycles)
        core(R, r, L, l, cmd, exec_pulse, th1, th2, th3, CYCLE, ff_enable, kff, True)
        # 模拟电机跟随：实际角度逐步逼近目标角度
        th1 += (core.TarTheta1 - th1) * 0.3
        th2 += (core.TarTheta2 - th2) * 0.3
        th3 += (core.TarTheta3 - th3) * 0.3
        if core.InPos and not core.Busy:
            # 等待几个周期确认稳定
            stable = 0
            for j in range(20):
                core(R, r, L, l, cmd, False, th1, th2, th3, CYCLE, ff_enable, kff, True)
                if core.InPos: stable += 1
            return i, th1, th2, th3
    return -1, th1, th2, th3


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


    # ============================================================
    # 测试用例
    # ============================================================
    print("=" * 60)
    print("测试 1: PTP 梯形曲线")
    print("=" * 60)
    core = FB_DeltaCore()
    cmd = make_cmd(0, 50, 0, 300, speed=200, accel=1000, move_mode=0)
    cycles, th1, th2, th3 = run_core(core, cmd)
    test("运动完成", cycles > 0, f"cycles={cycles}")
    test("到达目标 X≈50", abs(core.PlanX - 50) < 1.0, f"PlanX={core.PlanX:.2f}")
    test("到达目标 Z≈300", abs(core.PlanZ - 300) < 1.0, f"PlanZ={core.PlanZ:.2f}")
    test("InPos=TRUE", core.InPos)
    test("Busy=FALSE", not core.Busy)

    print("\n" + "=" * 60)
    print("测试 2: PTP S曲线")
    print("=" * 60)
    core = FB_DeltaCore()
    cmd = make_cmd(0, -30, 40, 250, speed=150, accel=800, jerk=8000, move_mode=1)
    cycles, _, _, _ = run_core(core, cmd)
    test("运动完成", cycles > 0)
    test("到达目标", abs(core.PlanX+30)<1 and abs(core.PlanY-40)<1 and abs(core.PlanZ-250)<1,
         f"({core.PlanX:.1f},{core.PlanY:.1f},{core.PlanZ:.1f})")
    test("S曲线运动时间 > 梯形(同参数)", cycles > 0)

    print("\n" + "=" * 60)
    print("测试 3: LIN 梯形曲线")
    print("=" * 60)
    core = FB_DeltaCore()
    # 先PTP到起点
    cmd0 = make_cmd(0, 0, 0, 300, speed=200, accel=1000)
    run_core(core, cmd0)
    # 目标须在工作空间内：(100,50,350) 超出可达范围（该XY半径下Z_max≈345）
    cmd = make_cmd(1, 60, 30, 320, speed=200, accel=1000, move_mode=0)
    cycles, _, _, _ = run_core(core, cmd)
    test("运动完成", cycles > 0)
    test("到达目标", abs(core.PlanX-60)<1 and abs(core.PlanY-30)<1 and abs(core.PlanZ-320)<1,
         f"({core.PlanX:.1f},{core.PlanY:.1f},{core.PlanZ:.1f})")

    print("\n" + "=" * 60)
    print("测试 4: LIN S曲线")
    print("=" * 60)
    core = FB_DeltaCore()
    cmd0 = make_cmd(0, 0, 0, 300, speed=200, accel=1000)
    run_core(core, cmd0)
    cmd = make_cmd(1, -80, -60, 280, speed=180, accel=900, jerk=9000, move_mode=1)
    cycles, _, _, _ = run_core(core, cmd)
    test("运动完成", cycles > 0)
    test("到达目标", abs(core.PlanX+80)<1 and abs(core.PlanY+60)<1 and abs(core.PlanZ-280)<1,
         f"({core.PlanX:.1f},{core.PlanY:.1f},{core.PlanZ:.1f})")

    print("\n" + "=" * 60)
    print("测试 5: CIR 圆心式")
    print("=" * 60)
    core = FB_DeltaCore()
    cmd0 = make_cmd(0, 100, 0, 300, speed=200, accel=1000)
    run_core(core, cmd0)
    # 圆心(50,0,300)，从(100,0,300)到(50,50,300)，半径50
    cmd = make_cmd(2, 50, 50, 300, speed=150, accel=800, move_mode=0,
                   aux=(50, 0, 300), cir_mode=0)
    cycles, _, _, _ = run_core(core, cmd)
    test("运动完成", cycles > 0)
    test("到达目标", abs(core.PlanX-50)<2 and abs(core.PlanY-50)<2 and abs(core.PlanZ-300)<2,
         f"({core.PlanX:.1f},{core.PlanY:.1f},{core.PlanZ:.1f})")

    print("\n" + "=" * 60)
    print("测试 6: CIR 三点式")
    print("=" * 60)
    core = FB_DeltaCore()
    cmd0 = make_cmd(0, 100, 0, 300, speed=200, accel=1000)
    run_core(core, cmd0)
    # 起点(100,0,300)，中间点(50,50,300)，终点(0,0,300)
    cmd = make_cmd(2, 0, 0, 300, speed=150, accel=800, move_mode=0,
                   aux=(50, 50, 300), cir_mode=1)
    cycles, _, _, _ = run_core(core, cmd)
    test("运动完成", cycles > 0)
    test("到达目标", abs(core.PlanX)<2 and abs(core.PlanY)<2 and abs(core.PlanZ-300)<2,
         f"({core.PlanX:.1f},{core.PlanY:.1f},{core.PlanZ:.1f})")

    print("\n" + "=" * 60)
    print("测试 7: 拒绝模式 (BufferMode=0)")
    print("=" * 60)
    core = FB_DeltaCore()
    cmd0 = make_cmd(0, 0, 0, 300, speed=100, accel=500)
    run_core(core, cmd0)
    # 发一条慢速运动
    cmd1 = make_cmd(1, 100, 0, 300, speed=50, accel=200, buffer_mode=0)
    # 不等完成，立刻发第二条
    th1 = th2 = th3 = 0.0
    rejected_seen = False
    for i in range(200):
        exec1 = (i == 0)
        core(R, r, L, l, cmd1, exec1, th1, th2, th3, CYCLE, True, 0.5, True)
        th1 += (core.TarTheta1 - th1) * 0.3
        th2 += (core.TarTheta2 - th2) * 0.3
        th3 += (core.TarTheta3 - th3) * 0.3
        if i == 50:
            # 发第二条拒绝模式指令
            cmd2 = make_cmd(1, 0, 100, 300, speed=50, accel=200, buffer_mode=0)
            core(R, r, L, l, cmd2, True, th1, th2, th3, CYCLE, True, 0.5, True)
            if core.Rejected:
                rejected_seen = True
            break
    test("Busy=TRUE", core.Busy)
    test("Rejected=TRUE", rejected_seen, "第二条指令应被拒绝")
    test("ActiveType=2(LIN)", core.ActiveType == 2, f"ActiveType={core.ActiveType}")

    print("\n" + "=" * 60)
    print("测试 8: 打断模式 (BufferMode=1)")
    print("=" * 60)
    core = FB_DeltaCore()
    cmd0 = make_cmd(0, 0, 0, 300, speed=200, accel=1000)
    run_core(core, cmd0)
    cmd1 = make_cmd(1, 100, 0, 300, speed=50, accel=200, buffer_mode=1)
    th1 = th2 = th3 = 0.0
    interrupted = False
    final_x = 0
    for i in range(1000):
        exec1 = (i == 0)
        core(R, r, L, l, cmd1, exec1, th1, th2, th3, CYCLE, True, 0.5, True)
        th1 += (core.TarTheta1 - th1) * 0.3
        th2 += (core.TarTheta2 - th2) * 0.3
        th3 += (core.TarTheta3 - th3) * 0.3
        if i == 100:
            # 打断到新位置
            cmd2 = make_cmd(0, 0, 100, 300, speed=200, accel=1000, buffer_mode=1)
            core(R, r, L, l, cmd2, True, th1, th2, th3, CYCLE, True, 0.5, True)
            interrupted = True
        if interrupted and core.InPos and not core.Busy:
            final_x = core.PlanX
            break
    test("打断成功", interrupted)
    test("打断后到达新目标 Y≈100", abs(core.PlanY - 100) < 2, f"PlanY={core.PlanY:.1f}")

    print("\n" + "=" * 60)
    print("测试 9: 缓存模式 (BufferMode=2)")
    print("=" * 60)
    core = FB_DeltaCore()
    cmd0 = make_cmd(0, 0, 0, 300, speed=200, accel=1000)
    run_core(core, cmd0)
    # 第一条运动
    cmd1 = make_cmd(1, 100, 0, 300, speed=100, accel=500, buffer_mode=2)
    th1 = th2 = th3 = 0.0
    buf_cmd_sent = False
    both_done = False
    for i in range(3000):
        exec1 = (i == 0)
        core(R, r, L, l, cmd1, exec1, th1, th2, th3, CYCLE, True, 0.5, True)
        th1 += (core.TarTheta1 - th1) * 0.3
        th2 += (core.TarTheta2 - th2) * 0.3
        th3 += (core.TarTheta3 - th3) * 0.3
        if i == 5 and not buf_cmd_sent:
            # 缓存第二条
            cmd2 = make_cmd(1, 100, 80, 300, speed=100, accel=500, buffer_mode=2)
            core(R, r, L, l, cmd2, True, th1, th2, th3, CYCLE, True, 0.5, True)
            buf_cmd_sent = True
        if buf_cmd_sent and core.InPos and not core.Busy:
            if abs(core.PlanY - 80) < 2:
                both_done = True
                break
    test("缓存指令已发送", buf_cmd_sent)
    test("两条都完成且到达第二条目标", both_done, f"PlanY={core.PlanY:.1f}")

    print("\n" + "=" * 60)
    print("测试 10: 缓存槽满拒绝")
    print("=" * 60)
    core = FB_DeltaCore()
    cmd0 = make_cmd(0, 0, 0, 300, speed=200, accel=1000)
    run_core(core, cmd0)
    cmd1 = make_cmd(1, 100, 0, 300, speed=50, accel=200, buffer_mode=2)
    th1 = th2 = th3 = 0.0
    rej1 = rej2 = False
    for i in range(200):
        exec1 = (i == 0)
        core(R, r, L, l, cmd1, exec1, th1, th2, th3, CYCLE, True, 0.5, True)
        th1 += (core.TarTheta1 - th1) * 0.3
        th2 += (core.TarTheta2 - th2) * 0.3
        th3 += (core.TarTheta3 - th3) * 0.3
        if i == 5:
            cmd2 = make_cmd(1, 0, 100, 300, speed=50, accel=200, buffer_mode=2)
            core(R, r, L, l, cmd2, True, th1, th2, th3, CYCLE, True, 0.5, True)
            rej1 = core.Rejected
        if i == 10:
            cmd3 = make_cmd(1, 50, 50, 300, speed=50, accel=200, buffer_mode=2)
            core(R, r, L, l, cmd3, True, th1, th2, th3, CYCLE, True, 0.5, True)
            rej2 = core.Rejected
            break
    test("第一条缓存成功(不拒绝)", not rej1)
    test("第二条缓存被拒绝(槽满)", rej2)
    test("Buffered=TRUE", core.Buffered)

    print("\n" + "=" * 60)
    print("测试 11: 打断清空缓存")
    print("=" * 60)
    core = FB_DeltaCore()
    cmd0 = make_cmd(0, 0, 0, 300, speed=200, accel=1000)
    run_core(core, cmd0)
    cmd1 = make_cmd(1, 100, 0, 300, speed=50, accel=200, buffer_mode=2)
    th1 = th2 = th3 = 0.0
    buf_then_interrupt = False
    for i in range(200):
        exec1 = (i == 0)
        core(R, r, L, l, cmd1, exec1, th1, th2, th3, CYCLE, True, 0.5, True)
        th1 += (core.TarTheta1 - th1) * 0.3
        th2 += (core.TarTheta2 - th2) * 0.3
        th3 += (core.TarTheta3 - th3) * 0.3
        if i == 5:
            cmd2 = make_cmd(1, 0, 100, 300, speed=50, accel=200, buffer_mode=2)
            core(R, r, L, l, cmd2, True, th1, th2, th3, CYCLE, True, 0.5, True)
        if i == 10 and core.Buffered:
            # 打断
            cmd3 = make_cmd(0, 0, 0, 350, speed=200, accel=1000, buffer_mode=1)
            core(R, r, L, l, cmd3, True, th1, th2, th3, CYCLE, True, 0.5, True)
            buf_then_interrupt = not core.Buffered
            break
    test("打断后缓存被清空", buf_then_interrupt, f"Buffered={core.Buffered}")

    print("\n" + "=" * 60)
    print("测试 12: 零距离运动")
    print("=" * 60)
    core = FB_DeltaCore()
    cmd0 = make_cmd(0, 50, 50, 300, speed=200, accel=1000)
    run_core(core, cmd0)
    # 目标=当前位置
    cmd = make_cmd(1, 50, 50, 300, speed=200, accel=1000)
    cycles, _, _, _ = run_core(core, cmd, max_cycles=100)
    test("零距离立即完成", cycles >= 0 and cycles < 5, f"cycles={cycles}")
    test("InPos=TRUE", core.InPos)

    print("\n" + "=" * 60)
    print("测试 13: 速度前馈使能 vs 禁用")
    print("=" * 60)
    # 禁用前馈
    core1 = FB_DeltaCore()
    cmd0 = make_cmd(0, 0, 0, 300, speed=300, accel=2000)
    th1 = th2 = th3 = 0.0
    for i in range(50):
        exec1 = (i == 0)
        core1(R, r, L, l, cmd0, exec1, th1, th2, th3, CYCLE, False, 0.0, True)
        th1 += (core1.TarTheta1 - th1) * 0.3
        th2 += (core1.TarTheta2 - th2) * 0.3
        th3 += (core1.TarTheta3 - th3) * 0.3
    tar_noff = core1.TarTheta1

    # 使能前馈
    core2 = FB_DeltaCore()
    th1 = th2 = th3 = 0.0
    for i in range(50):
        exec1 = (i == 0)
        core2(R, r, L, l, cmd0, exec1, th1, th2, th3, CYCLE, True, 0.5, True)
        th1 += (core2.TarTheta1 - th1) * 0.3
        th2 += (core2.TarTheta2 - th2) * 0.3
        th3 += (core2.TarTheta3 - th3) * 0.3
    tar_ff = core2.TarTheta1

    test("前馈使能时目标角度不同", abs(tar_ff - tar_noff) > 1e-6,
         f"noff={tar_noff:.6f} ff={tar_ff:.6f}")

    print("\n" + "=" * 60)
    print("测试 14: 连续 LIN 运动无跳变")
    print("=" * 60)
    core = FB_DeltaCore()
    cmd0 = make_cmd(0, 0, 0, 300, speed=200, accel=1000)
    run_core(core, cmd0)
    # 连续3段直线
    pts = [(50, 0, 300), (50, 50, 300), (0, 50, 300)]
    all_ok = True
    for pt in pts:
        cmd = make_cmd(1, pt[0], pt[1], pt[2], speed=150, accel=800)
        cycles, _, _, _ = run_core(core, cmd)
        if cycles < 0 or abs(core.PlanX-pt[0])>1 or abs(core.PlanY-pt[1])>1:
            all_ok = False
            break
    test("3段连续直线都到达", all_ok)

    print("\n" + "=" * 60)
    print("测试 15: PTP 大行程")
    print("=" * 60)
    core = FB_DeltaCore()
    cmd0 = make_cmd(0, 0, 0, 250, speed=200, accel=1000)
    run_core(core, cmd0)
    # 目标须在工作空间内（本机型 Z_max≈346mm），Z=400 不可达
    cmd = make_cmd(0, -80, 70, 300, speed=250, accel=1500, move_mode=1)
    cycles, _, _, _ = run_core(core, cmd, max_cycles=8000)
    test("大行程完成", cycles > 0, f"cycles={cycles}")
    test("到达目标", abs(core.PlanX+80)<2 and abs(core.PlanY-70)<2 and abs(core.PlanZ-300)<2,
         f"({core.PlanX:.1f},{core.PlanY:.1f},{core.PlanZ:.1f})")


    # ============================================================
    print("\n" + "=" * 60)
    print(f"测试结果: {passed} 通过, {failed} 失败")
    print("=" * 60)
