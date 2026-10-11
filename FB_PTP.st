(* ============================================================================
   功能块名称 : FB_PTP
   功能描述 : Delta 机器人点到点运动指令 FB（PLCopen 风格，关节空间同步插补）
              3轴共用归一化进度因子 s∈[0,1]，每周期角度组合合法：
              θ(t) = θ_cur + (θ* - θ_cur) × s
              s 按加减速曲线（梯形/S曲线）从 0 走到 1
   调用方式 : 每周期先调用各指令FB，最后调用 FB_DeltaCore
              Execute 上升沿启动；多指令FB共享同一 Axis(VAR_IN_OUT) 协调
   BufferMode: 0=拒绝(运动中直接 Rejected) 1=打断(抢占当前指令)
               2=缓存(当前完成后自动执行，单缓存槽，槽满 Rejected)
   输出语义  : Busy 含缓存等待期；InPos/CommandAborted 锁存到下次上升沿；
               Rejected 为单周期脉冲
   ============================================================================ *)

FUNCTION_BLOCK FB_PTP

VAR_INPUT
    Execute      : BOOL;    (* 启动（上升沿有效）                       *)
    PositionX    : REAL;    (* 目标位置 X (mm)                          *)
    PositionY    : REAL;    (* 目标位置 Y                               *)
    PositionZ    : REAL;    (* 目标位置 Z                               *)
    Speed        : REAL;    (* 等效速度 (mm/s)，决定总运动时间          *)
    Accel        : REAL;    (* 等效加速度 (mm/s²)                       *)
    Jerk         : REAL;    (* 等效加加速度 (mm/s³)，S曲线用            *)
    MoveMode     : INT;     (* 加减速模式：0=梯形 1=S曲线               *)
    BufferMode   : INT;     (* 缓冲模式：0=拒绝 1=打断 2=缓存           *)
END_VAR

VAR_IN_OUT
    Axis         : st_DeltaAxis;   (* 共享轴状态                        *)
END_VAR

VAR_OUTPUT
    Busy            : BOOL;    (* 执行中或缓存等待中                     *)
    InPos           : BOOL;    (* 已到达（锁存）                         *)
    CommandAborted  : BOOL;    (* 被其他指令打断（锁存）                 *)
    Rejected        : BOOL;    (* 指令被拒绝（单周期脉冲）               *)
END_VAR

VAR
    (* 常量 *)
    COS1:REAL:=1.0; SIN1:REAL:=0.0;
    COS2:REAL:=-0.5; SIN2:REAL:=0.8660254037844386;
    COS3:REAL:=-0.5; SIN3:REAL:=-0.8660254037844386;

    (* IK中间变量 *)
    xp,yp,A,B,C,Disc,u1,u2,ts1,ts2,denom:REAL;

    (* FK中间变量 *)
    s1,s2,s3,sn1,sn2,sn3,a1,b1,c1,d1,a2,b2,c2,d2,Det,px,py,qx,qy,X0,Y0,Az,Bz,Cz,Dz,z1,z2:REAL;

    (* PTP轨迹状态 *)
    e:REAL;
    curX,curY,curZ:REAL;        (* 启动时当前位置(FK) *)
    tgtX,tgtY,tgtZ:REAL;        (* 目标位置(启动时存入) *)
    tgtTh1,tgtTh2,tgtTh3:REAL;  (* 目标角度(IK) *)
    startTh1,startTh2,startTh3:REAL; (* 起始角度 *)
    totalDist:REAL;             (* 总距离 *)
    s:REAL;                     (* 归一化进度 0~1 *)
    sVel:REAL;                  (* s的速度 *)
    sAcc:REAL;                  (* s的加速度 *)
    maxSVel:REAL;               (* s的最大速度 = Speed/totalDist *)
    maxSAcc:REAL;               (* s的最大加速度 = Accel/totalDist *)
    maxSJerk:REAL;              (* s的最大加加速度 = Jerk/totalDist *)
    sStop:REAL;                 (* s的减速距离估计 *)
    sRem:REAL;                  (* s的剩余距离 = 1 - s *)
    phase:INT;                  (* 0=idle 1=accel 2=const 3=decel 4=done *)
    interpTh1,interpTh2,interpTh3:REAL; (* 插值角度 *)

    (* 调度状态 *)
    myId:DINT;                  (* 本FB持有的控制权令牌 *)
    running:BOOL;               (* 本指令正在执行 *)
    waiting:BOOL;               (* 本指令在缓存槽等待 *)
    wantStart:BOOL;             (* 本周期请求启动（统一入口） *)
    exLast:BOOL;
    exRise:BOOL;
END_VAR


(* ============================================================================
   失能：复位自身，不写 Axis 规划
   ============================================================================ *)
IF NOT Axis.Enable THEN
    Busy:=FALSE; InPos:=FALSE; CommandAborted:=FALSE; Rejected:=FALSE;
    running:=FALSE; waiting:=FALSE; phase:=0; s:=0.0;
    exLast:=Execute;
    RETURN;
END_IF;

Rejected:=FALSE;

(* Execute 上升沿检测 *)
exRise := Execute AND NOT exLast;
exLast := Execute;

wantStart := FALSE;

(* ============================================================================
   第一步：上升沿仲裁
   ============================================================================ *)
IF exRise THEN
    InPos:=FALSE; CommandAborted:=FALSE; waiting:=FALSE;

    IF (Speed <= 0.0) OR (Accel <= 0.0) OR ((MoveMode = 1) AND (Jerk <= 0.0)) THEN
        (* 参数非法：拒绝，避免除零或卡死 *)
        Rejected:=TRUE;
    ELSIF running AND (Axis.owner = myId) THEN
        (* 同实例重触发（打断自己）：无条件从当前位置重启 *)
        wantStart:=TRUE;
    ELSIF Axis.Busy THEN
        CASE BufferMode OF
            0:  (* 拒绝 *)
                Rejected:=TRUE;
            1:  (* 打断：抢占当前指令 *)
                wantStart:=TRUE;
            2:  (* 缓存：单槽，槽满则拒绝 *)
                IF Axis.hasBuf THEN
                    Rejected:=TRUE;
                ELSE
                    waiting:=TRUE; Axis.hasBuf:=TRUE; Busy:=TRUE;
                END_IF;
        END_CASE;
    ELSE
        (* 轴空闲：直接启动 *)
        wantStart:=TRUE;
    END_IF;
END_IF;

(* ============================================================================
   第二步：缓存等待处理
   ============================================================================ *)
IF waiting THEN
    IF NOT Axis.hasBuf THEN
        (* 缓存槽被打断指令清空：本缓存指令作废 *)
        waiting:=FALSE; Busy:=FALSE; CommandAborted:=TRUE;
    ELSIF NOT Axis.Busy THEN
        (* 前一条已完成，接管轴控制权 *)
        waiting:=FALSE; Axis.hasBuf:=FALSE; wantStart:=TRUE;
    END_IF;
END_IF;

(* ============================================================================
   第三步：统一启动（令牌 + 轨迹初始化）
   ============================================================================ *)
IF wantStart THEN
    myId:=Axis.cmdCount + 1;
    Axis.cmdCount:=myId;
    Axis.owner:=myId;
    Axis.ActiveType:=1;
    Axis.Busy:=TRUE;
    Axis.InPos:=FALSE;
    Axis.hasBuf:=FALSE;
    running:=TRUE; Busy:=TRUE; InPos:=FALSE;

    e := Axis.R - Axis.r;

    (* --- FK: 当前角度 → 当前位置 --- *)
    s1:=e+Axis.L*COS(Axis.CurTheta1); s2:=e+Axis.L*COS(Axis.CurTheta2); s3:=e+Axis.L*COS(Axis.CurTheta3);
    sn1:=Axis.L*SIN(Axis.CurTheta1); sn2:=Axis.L*SIN(Axis.CurTheta2); sn3:=Axis.L*SIN(Axis.CurTheta3);
    a1:=-2.0*(s2*COS2-s1*COS1); b1:=-2.0*(s2*SIN2-s1*SIN1);
    c1:=-2.0*(sn2-sn1); d1:=(s1*s1+sn1*sn1)-(s2*s2+sn2*sn2);
    a2:=-2.0*(s3*COS3-s1*COS1); b2:=-2.0*(s3*SIN3-s1*SIN1);
    c2:=-2.0*(sn3-sn1); d2:=(s1*s1+sn1*sn1)-(s3*s3+sn3*sn3);
    Det:=a1*b2-a2*b1;
    IF ABS(Det)>1.0E-9 THEN
        px:=(b2*d1-b1*d2)/Det; qx:=(b1*c2-b2*c1)/Det;
        py:=(a1*d2-a2*d1)/Det; qy:=(a2*c1-a1*c2)/Det;
        X0:=px-s1*COS1; Y0:=py-s1*SIN1;
        Az:=qx*qx+qy*qy+1.0; Bz:=2.0*(X0*qx+Y0*qy)-2.0*sn1;
        Cz:=X0*X0+Y0*Y0+sn1*sn1-Axis.l*Axis.l; Dz:=Bz*Bz-4.0*Az*Cz;
        IF Dz>=0.0 THEN
            z1:=(-Bz+SQRT(Dz))/(2.0*Az); z2:=(-Bz-SQRT(Dz))/(2.0*Az);
            IF z1>0.0 THEN curZ:=z1; ELSIF z2>0.0 THEN curZ:=z2; ELSE curZ:=z1; END_IF;
            curX:=px+qx*curZ; curY:=py+qy*curZ;
        ELSE
            curX:=0.0; curY:=0.0; curZ:=0.0;
        END_IF;
    ELSE
        curX:=0.0; curY:=0.0; curZ:=0.0;
    END_IF;

    (* --- IK: 目标位置 → 目标角度 --- *)
    tgtX := PositionX; tgtY := PositionY; tgtZ := PositionZ;
    (* 轴1 *)
    xp:=PositionX*COS1+PositionY*SIN1; yp:=-PositionX*SIN1+PositionY*COS1;
    A:=2.0*Axis.L*(e-xp); B:=-2.0*Axis.L*PositionZ;
    C:=Axis.L*Axis.L+(e-xp)*(e-xp)+yp*yp+PositionZ*PositionZ-Axis.l*Axis.l;
    Disc:=A*A+B*B-C*C;
    IF Disc>=0.0 THEN
        denom:=C-A;
        IF ABS(denom)>1.0E-9 THEN
            u1:=(-B+SQRT(Disc))/denom; u2:=(-B-SQRT(Disc))/denom;
            ts1:=2.0*ATAN(u1); ts2:=2.0*ATAN(u2);
            IF ts1>0.0 AND ts2>0.0 THEN
                IF ABS(ts1-Axis.CurTheta1)<=ABS(ts2-Axis.CurTheta1) THEN tgtTh1:=ts1; ELSE tgtTh1:=ts2; END_IF;
            ELSIF ts1>0.0 THEN tgtTh1:=ts1;
            ELSIF ts2>0.0 THEN tgtTh1:=ts2;
            ELSE tgtTh1:=ts1; END_IF;
        ELSE tgtTh1:=Axis.CurTheta1; END_IF;
    ELSE tgtTh1:=Axis.CurTheta1; END_IF;

    (* 轴2 *)
    xp:=PositionX*COS2+PositionY*SIN2; yp:=-PositionX*SIN2+PositionY*COS2;
    A:=2.0*Axis.L*(e-xp); B:=-2.0*Axis.L*PositionZ;
    C:=Axis.L*Axis.L+(e-xp)*(e-xp)+yp*yp+PositionZ*PositionZ-Axis.l*Axis.l;
    Disc:=A*A+B*B-C*C;
    IF Disc>=0.0 THEN
        denom:=C-A;
        IF ABS(denom)>1.0E-9 THEN
            u1:=(-B+SQRT(Disc))/denom; u2:=(-B-SQRT(Disc))/denom;
            ts1:=2.0*ATAN(u1); ts2:=2.0*ATAN(u2);
            IF ts1>0.0 AND ts2>0.0 THEN
                IF ABS(ts1-Axis.CurTheta2)<=ABS(ts2-Axis.CurTheta2) THEN tgtTh2:=ts1; ELSE tgtTh2:=ts2; END_IF;
            ELSIF ts1>0.0 THEN tgtTh2:=ts1;
            ELSIF ts2>0.0 THEN tgtTh2:=ts2;
            ELSE tgtTh2:=ts1; END_IF;
        ELSE tgtTh2:=Axis.CurTheta2; END_IF;
    ELSE tgtTh2:=Axis.CurTheta2; END_IF;

    (* 轴3 *)
    xp:=PositionX*COS3+PositionY*SIN3; yp:=-PositionX*SIN3+PositionY*COS3;
    A:=2.0*Axis.L*(e-xp); B:=-2.0*Axis.L*PositionZ;
    C:=Axis.L*Axis.L+(e-xp)*(e-xp)+yp*yp+PositionZ*PositionZ-Axis.l*Axis.l;
    Disc:=A*A+B*B-C*C;
    IF Disc>=0.0 THEN
        denom:=C-A;
        IF ABS(denom)>1.0E-9 THEN
            u1:=(-B+SQRT(Disc))/denom; u2:=(-B-SQRT(Disc))/denom;
            ts1:=2.0*ATAN(u1); ts2:=2.0*ATAN(u2);
            IF ts1>0.0 AND ts2>0.0 THEN
                IF ABS(ts1-Axis.CurTheta3)<=ABS(ts2-Axis.CurTheta3) THEN tgtTh3:=ts1; ELSE tgtTh3:=ts2; END_IF;
            ELSIF ts1>0.0 THEN tgtTh3:=ts1;
            ELSIF ts2>0.0 THEN tgtTh3:=ts2;
            ELSE tgtTh3:=ts1; END_IF;
        ELSE tgtTh3:=Axis.CurTheta3; END_IF;
    ELSE tgtTh3:=Axis.CurTheta3; END_IF;

    (* --- 初始化进度 s --- *)
    startTh1:=Axis.CurTheta1; startTh2:=Axis.CurTheta2; startTh3:=Axis.CurTheta3;
    totalDist:=SQRT((PositionX-curX)*(PositionX-curX)+(PositionY-curY)*(PositionY-curY)+(PositionZ-curZ)*(PositionZ-curZ));
    IF totalDist < 1.0E-4 THEN
        (* 已在目标位置：当周期完成 *)
        Axis.PlanX:=tgtX; Axis.PlanY:=tgtY; Axis.PlanZ:=tgtZ;
        Axis.PlanVX:=0.0; Axis.PlanVY:=0.0; Axis.PlanVZ:=0.0;
        s:=1.0; phase:=4;
    ELSE
        maxSVel:=Speed/totalDist;
        maxSAcc:=Accel/totalDist;
        maxSJerk:=Jerk/totalDist;
        s:=0.0; sVel:=0.0; sAcc:=0.0; phase:=1;
        Axis.InPos:=FALSE;
    END_IF;
END_IF;

(* ============================================================================
   第四步：被其他指令打断（令牌易主）→ 停止写规划
   ============================================================================ *)
IF running AND (Axis.owner <> myId) THEN
    running:=FALSE; phase:=0; Busy:=FALSE; CommandAborted:=TRUE;
END_IF;

(* ============================================================================
   第五步：执行中轨迹计算 → 写 Axis.Plan*
   ============================================================================ *)
IF running AND (phase = 1 OR phase = 2 OR phase = 3) THEN

    sRem := 1.0 - s;

    (* 减速距离估计 *)
    IF MoveMode = 0 THEN
        sStop := sVel * sVel / (2.0 * maxSAcc);
    ELSE
        sStop := sVel*sVel/(2.0*maxSAcc) + sVel*maxSAcc/(2.0*maxSJerk);
    END_IF;

    CASE phase OF
        1:  (* 加速 *)
            IF sRem <= sStop THEN
                phase := 3;
            ELSE
                IF MoveMode = 0 THEN
                    sAcc := maxSAcc;
                ELSE
                    sAcc := sAcc + maxSJerk * Axis.CycleTime;
                    IF sAcc > maxSAcc THEN sAcc := maxSAcc; END_IF;
                END_IF;
                sVel := sVel + sAcc * Axis.CycleTime;
                IF sVel >= maxSVel THEN
                    sVel := maxSVel; sAcc := 0.0; phase := 2;
                END_IF;
            END_IF;

        2:  (* 匀速 *)
            IF sRem <= sStop THEN
                phase := 3;
            ELSE
                sVel := maxSVel; sAcc := 0.0;
            END_IF;

        3:  (* 减速 *)
            IF MoveMode = 0 THEN
                sAcc := -maxSAcc;
            ELSE
                sAcc := sAcc - maxSJerk * Axis.CycleTime;
                IF sAcc < -maxSAcc THEN sAcc := -maxSAcc; END_IF;
            END_IF;
            sVel := sVel + sAcc * Axis.CycleTime;
            IF sVel <= 0.0 THEN
                sVel := 0.0; sAcc := 0.0;
                s := 1.0; phase := 4;
            END_IF;
    END_CASE;

    IF phase <> 4 THEN
        s := s + sVel * Axis.CycleTime;
        IF s > 1.0 THEN s := 1.0; END_IF;
    END_IF;

    (* 3轴同步插值：θ(t) = θ_start + (θ_target - θ_start) × s *)
    interpTh1 := startTh1 + (tgtTh1 - startTh1) * s;
    interpTh2 := startTh2 + (tgtTh2 - startTh2) * s;
    interpTh3 := startTh3 + (tgtTh3 - startTh3) * s;

    (* FK: 插值角度 → 规划位置 *)
    s1:=e+Axis.L*COS(interpTh1); s2:=e+Axis.L*COS(interpTh2); s3:=e+Axis.L*COS(interpTh3);
    sn1:=Axis.L*SIN(interpTh1); sn2:=Axis.L*SIN(interpTh2); sn3:=Axis.L*SIN(interpTh3);
    a1:=-2.0*(s2*COS2-s1*COS1); b1:=-2.0*(s2*SIN2-s1*SIN1);
    c1:=-2.0*(sn2-sn1); d1:=(s1*s1+sn1*sn1)-(s2*s2+sn2*sn2);
    a2:=-2.0*(s3*COS3-s1*COS1); b2:=-2.0*(s3*SIN3-s1*SIN1);
    c2:=-2.0*(sn3-sn1); d2:=(s1*s1+sn1*sn1)-(s3*s3+sn3*sn3);
    Det:=a1*b2-a2*b1;
    IF ABS(Det)>1.0E-9 THEN
        px:=(b2*d1-b1*d2)/Det; qx:=(b1*c2-b2*c1)/Det;
        py:=(a1*d2-a2*d1)/Det; qy:=(a2*c1-a1*c2)/Det;
        X0:=px-s1*COS1; Y0:=py-s1*SIN1;
        Az:=qx*qx+qy*qy+1.0; Bz:=2.0*(X0*qx+Y0*qy)-2.0*sn1;
        Cz:=X0*X0+Y0*Y0+sn1*sn1-Axis.l*Axis.l; Dz:=Bz*Bz-4.0*Az*Cz;
        IF Dz>=0.0 THEN
            z1:=(-Bz+SQRT(Dz))/(2.0*Az); z2:=(-Bz-SQRT(Dz))/(2.0*Az);
            IF z1>0.0 THEN Axis.PlanZ:=z1; ELSIF z2>0.0 THEN Axis.PlanZ:=z2; ELSE Axis.PlanZ:=z1; END_IF;
            Axis.PlanX:=px+qx*Axis.PlanZ; Axis.PlanY:=py+qy*Axis.PlanZ;
        END_IF;
    END_IF;

    (* 规划速度 = (目标-启动时当前位置) × ṡ *)
    Axis.PlanVX := (tgtX - curX) * sVel;
    Axis.PlanVY := (tgtY - curY) * sVel;
    Axis.PlanVZ := (tgtZ - curZ) * sVel;

    IF phase = 4 THEN
        (* Plan 保持为最终插值角度的 FK 结果（≈目标，误差为FK数值精度），
           不强制写为目标值：目标不可达时避免虚报位置 *)
        Axis.PlanVX := 0.0; Axis.PlanVY := 0.0; Axis.PlanVZ := 0.0;
    END_IF;
END_IF;

(* ============================================================================
   第六步：完成 → 释放轴控制权
   ============================================================================ *)
IF running AND (phase = 4) THEN
    running:=FALSE;
    Busy:=FALSE; InPos:=TRUE;
    Axis.Busy:=FALSE; Axis.ActiveType:=0; Axis.owner:=0; Axis.InPos:=TRUE;
END_IF;

END_FUNCTION_BLOCK
