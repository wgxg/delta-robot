(* ============================================================================
   功能块名称 : FB_PTP
   功能描述 : Delta 机器人点到点运动（关节空间同步插补）
              核心：3轴共用归一化进度因子 s∈[0,1]，每周期角度组合合法
              θ(t) = θ_cur + (θ* - θ_cur) × s
              s 按加减速曲线（梯形/S曲线）从 0 走到 1
   调用方式 : 周期调用，Execute 上升沿启动
   ============================================================================ *)

FUNCTION_BLOCK FB_PTP

VAR_INPUT
    R, r, L, l   : REAL;    (* 机械参数                              *)
    TargetX      : REAL;    (* 目标位置 X (mm)                       *)
    TargetY      : REAL;    (* 目标位置 Y                             *)
    TargetZ      : REAL;    (* 目标位置 Z                             *)
    CurTheta1    : REAL;    (* 当前角度 (rad)                        *)
    CurTheta2    : REAL;
    CurTheta3    : REAL;
    Speed        : REAL;    (* 等效速度 (mm/s)，决定总运动时间       *)
    Accel        : REAL;    (* 等效加速度 (mm/s²)                    *)
    Jerk         : REAL;    (* 等效加加速度 (mm/s³)，S曲线用         *)
    MoveMode     : INT;     (* 0=梯形 1=S曲线                        *)
    CycleTime    : REAL;    (* 周期 (s)                              *)
    Enable       : BOOL;
    Execute      : BOOL;    (* 启动脉冲                              *)
END_VAR

VAR_OUTPUT
    PlanX, PlanY, PlanZ : REAL;  (* 规划位置 (mm)                  *)
    PlanVX, PlanVY, PlanVZ : REAL; (* 规划速度 (mm/s)             *)
    InPos        : BOOL;    (* 已到达                                *)
    Busy         : BOOL;    (* 运动中                                *)
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

    (* PTP状态 *)
    e:REAL;
    curX,curY,curZ:REAL;        (* 当前位置(FK) *)
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
    exLast:BOOL;                (* 上次Execute *)
    exRise:BOOL;                (* Execute上升沿 *)
    interpTh1,interpTh2,interpTh3:REAL; (* 插值角度 *)
END_VAR


IF NOT Enable THEN
    PlanX:=0.0; PlanY:=0.0; PlanZ:=0.0;
    PlanVX:=0.0; PlanVY:=0.0; PlanVZ:=0.0;
    InPos:=FALSE; Busy:=FALSE; phase:=0; s:=0.0;
    RETURN;
END_IF;

e := R - r;

(* Execute 上升沿检测 *)
exRise := Execute AND NOT exLast;
exLast := Execute;


(* ============================================================================
   启动：FK当前位置 + IK目标角度 + 初始化s
   ============================================================================ *)
IF exRise THEN
    (* --- FK: 当前角度 → 当前位置 --- *)
    s1:=e+L*COS(CurTheta1); s2:=e+L*COS(CurTheta2); s3:=e+L*COS(CurTheta3);
    sn1:=L*SIN(CurTheta1); sn2:=L*SIN(CurTheta2); sn3:=L*SIN(CurTheta3);
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
        Cz:=X0*X0+Y0*Y0+sn1*sn1-l*l; Dz:=Bz*Bz-4.0*Az*Cz;
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
    tgtX := TargetX; tgtY := TargetY; tgtZ := TargetZ;
    (* 轴1 *)
    xp:=TargetX*COS1+TargetY*SIN1; yp:=-TargetX*SIN1+TargetY*COS1;
    A:=2.0*L*(e-xp); B:=-2.0*L*TargetZ;
    C:=L*L+(e-xp)*(e-xp)+yp*yp+TargetZ*TargetZ-l*l;
    Disc:=A*A+B*B-C*C;
    IF Disc>=0.0 THEN
        denom:=C-A;
        IF ABS(denom)>1.0E-9 THEN
            u1:=(-B+SQRT(Disc))/denom; u2:=(-B-SQRT(Disc))/denom;
            ts1:=2.0*ATAN(u1); ts2:=2.0*ATAN(u2);
            IF ts1>0.0 AND ts2>0.0 THEN
                IF ABS(ts1-CurTheta1)<=ABS(ts2-CurTheta1) THEN tgtTh1:=ts1; ELSE tgtTh1:=ts2; END_IF;
            ELSIF ts1>0.0 THEN tgtTh1:=ts1;
            ELSIF ts2>0.0 THEN tgtTh1:=ts2;
            ELSE tgtTh1:=ts1; END_IF;
        ELSE tgtTh1:=CurTheta1; END_IF;
    ELSE tgtTh1:=CurTheta1; END_IF;

    (* 轴2 *)
    xp:=TargetX*COS2+TargetY*SIN2; yp:=-TargetX*SIN2+TargetY*COS2;
    A:=2.0*L*(e-xp); B:=-2.0*L*TargetZ;
    C:=L*L+(e-xp)*(e-xp)+yp*yp+TargetZ*TargetZ-l*l;
    Disc:=A*A+B*B-C*C;
    IF Disc>=0.0 THEN
        denom:=C-A;
        IF ABS(denom)>1.0E-9 THEN
            u1:=(-B+SQRT(Disc))/denom; u2:=(-B-SQRT(Disc))/denom;
            ts1:=2.0*ATAN(u1); ts2:=2.0*ATAN(u2);
            IF ts1>0.0 AND ts2>0.0 THEN
                IF ABS(ts1-CurTheta2)<=ABS(ts2-CurTheta2) THEN tgtTh2:=ts1; ELSE tgtTh2:=ts2; END_IF;
            ELSIF ts1>0.0 THEN tgtTh2:=ts1;
            ELSIF ts2>0.0 THEN tgtTh2:=ts2;
            ELSE tgtTh2:=ts1; END_IF;
        ELSE tgtTh2:=CurTheta2; END_IF;
    ELSE tgtTh2:=CurTheta2; END_IF;

    (* 轴3 *)
    xp:=TargetX*COS3+TargetY*SIN3; yp:=-TargetX*SIN3+TargetY*COS3;
    A:=2.0*L*(e-xp); B:=-2.0*L*TargetZ;
    C:=L*L+(e-xp)*(e-xp)+yp*yp+TargetZ*TargetZ-l*l;
    Disc:=A*A+B*B-C*C;
    IF Disc>=0.0 THEN
        denom:=C-A;
        IF ABS(denom)>1.0E-9 THEN
            u1:=(-B+SQRT(Disc))/denom; u2:=(-B-SQRT(Disc))/denom;
            ts1:=2.0*ATAN(u1); ts2:=2.0*ATAN(u2);
            IF ts1>0.0 AND ts2>0.0 THEN
                IF ABS(ts1-CurTheta3)<=ABS(ts2-CurTheta3) THEN tgtTh3:=ts1; ELSE tgtTh3:=ts2; END_IF;
            ELSIF ts1>0.0 THEN tgtTh3:=ts1;
            ELSIF ts2>0.0 THEN tgtTh3:=ts2;
            ELSE tgtTh3:=ts1; END_IF;
        ELSE tgtTh3:=CurTheta3; END_IF;
    ELSE tgtTh3:=CurTheta3; END_IF;

    (* --- 初始化进度 s --- *)
    startTh1:=CurTheta1; startTh2:=CurTheta2; startTh3:=CurTheta3;
    totalDist:=SQRT((TargetX-curX)*(TargetX-curX)+(TargetY-curY)*(TargetY-curY)+(TargetZ-curZ)*(TargetZ-curZ));
    IF totalDist < 1.0E-4 THEN
        (* 已在目标位置 *)
        PlanX:=tgtX; PlanY:=tgtY; PlanZ:=tgtZ;
        s:=1.0; phase:=4; InPos:=TRUE; Busy:=FALSE;
    ELSE
        maxSVel:=Speed/totalDist;
        maxSAcc:=Accel/totalDist;
        maxSJerk:=Jerk/totalDist;
        s:=0.0; sVel:=0.0; sAcc:=0.0; phase:=1;
        InPos:=FALSE; Busy:=TRUE;
    END_IF;
END_IF;


(* ============================================================================
   每周期：更新 s（加减速）+ 插值角度 + FK得到规划位置
   ============================================================================ *)
IF phase <> 4 AND phase <> 0 THEN

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
                    sAcc := sAcc + maxSJerk * CycleTime;
                    IF sAcc > maxSAcc THEN sAcc := maxSAcc; END_IF;
                END_IF;
                sVel := sVel + sAcc * CycleTime;
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
                sAcc := sAcc - maxSJerk * CycleTime;
                IF sAcc < -maxSAcc THEN sAcc := -maxSAcc; END_IF;
            END_IF;
            sVel := sVel + sAcc * CycleTime;
            IF sVel <= 0.0 THEN
                sVel := 0.0; sAcc := 0.0;
                s := 1.0; phase := 4;
            END_IF;
    END_CASE;

    IF phase <> 4 THEN
        s := s + sVel * CycleTime;
        IF s > 1.0 THEN s := 1.0; END_IF;
    END_IF;

    (* 3轴同步插值：θ(t) = θ_start + (θ_target - θ_start) × s *)
    interpTh1 := startTh1 + (tgtTh1 - startTh1) * s;
    interpTh2 := startTh2 + (tgtTh2 - startTh2) * s;
    interpTh3 := startTh3 + (tgtTh3 - startTh3) * s;

    (* FK: 插值角度 → 规划位置 *)
    s1:=e+L*COS(interpTh1); s2:=e+L*COS(interpTh2); s3:=e+L*COS(interpTh3);
    sn1:=L*SIN(interpTh1); sn2:=L*SIN(interpTh2); sn3:=L*SIN(interpTh3);
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
        Cz:=X0*X0+Y0*Y0+sn1*sn1-l*l; Dz:=Bz*Bz-4.0*Az*Cz;
        IF Dz>=0.0 THEN
            z1:=(-Bz+SQRT(Dz))/(2.0*Az); z2:=(-Bz-SQRT(Dz))/(2.0*Az);
            IF z1>0.0 THEN PlanZ:=z1; ELSIF z2>0.0 THEN PlanZ:=z2; ELSE PlanZ:=z1; END_IF;
            PlanX:=px+qx*PlanZ; PlanY:=py+qy*PlanZ;
        END_IF;
    END_IF;

    (* 规划速度 = (目标-当前位置) × ṡ *)
    PlanVX := (tgtX - curX) * sVel;
    PlanVY := (tgtY - curY) * sVel;
    PlanVZ := (tgtZ - curZ) * sVel;

    IF phase = 4 THEN
        PlanX := tgtX; PlanY := tgtY; PlanZ := tgtZ;
        PlanVX := 0.0; PlanVY := 0.0; PlanVZ := 0.0;
        InPos := TRUE; Busy := FALSE;
    END_IF;
END_IF;

END_FUNCTION_BLOCK
