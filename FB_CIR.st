(* ============================================================================
   功能块名称 : FB_CIR
   功能描述 : Delta 机器人笛卡尔空间圆弧插补
              支持两种模式：
                CirMode=0 圆心式：起点(当前Plan) + 终点(Target) + 圆心(Aux)
                CirMode=1 三点式：起点(当前Plan) + 中间点(Aux) + 终点(Target)
              按弧长均匀插补，加减速在弧长方向
   调用方式 : 周期调用，Execute 上升沿启动
   ============================================================================ *)

FUNCTION_BLOCK FB_CIR

VAR_INPUT
    TargetX      : REAL;    (* 终点位置 X (mm)                       *)
    TargetY      : REAL;
    TargetZ      : REAL;
    AuxX         : REAL;    (* 辅助点 X：圆心式=圆心，三点式=中间点   *)
    AuxY         : REAL;
    AuxZ         : REAL;
    CirMode      : INT;     (* 0=圆心式  1=三点式                    *)
    Speed        : REAL;    (* 最大速度 (mm/s)                        *)
    Accel        : REAL;    (* 最大加速度 (mm/s²)                     *)
    Jerk         : REAL;    (* 最大加加速度 (mm/s³)，S曲线用          *)
    MoveMode     : INT;     (* 0=梯形 1=S曲线                         *)
    CycleTime    : REAL;    (* 周期 (s)                               *)
    Enable       : BOOL;
    Execute      : BOOL;    (* 启动脉冲                               *)
    InitX        : REAL;    (* 初始位置 X（首次启用时用）             *)
    InitY        : REAL;
    InitZ        : REAL;
END_VAR

VAR_OUTPUT
    PlanX, PlanY, PlanZ : REAL;    (* 规划位置 (mm)                *)
    PlanVX, PlanVY, PlanVZ : REAL; (* 规划速度 (mm/s)             *)
    InPos        : BOOL;    (* 已到达                                *)
    Busy         : BOOL;    (* 运动中                                *)
END_VAR

VAR
    (* 圆弧几何参数 *)
    cx,cy,cz:REAL;          (* 圆心 *)
    radius:REAL;            (* 半径 *)
    ux,uy,uz:REAL;          (* 平面基向量u（圆心→起点方向） *)
    vx,vy,vz:REAL;          (* 平面基向量v（u×法向量） *)
    thetaStart:REAL;        (* 起始角 *)
    thetaEnd:REAL;          (* 终止角 *)
    thetaSpan:REAL;         (* 圆弧夹角 (rad) *)
    arcLen:REAL;            (* 弧长 *)
    maxSVel:REAL;           (* 进度域最大速度 = Speed/arcLen (1/s) *)
    maxSAcc:REAL;           (* 进度域最大加速度 (1/s²) *)
    maxSJerk:REAL;          (* 进度域最大加加速度 (1/s³) *)

    (* 运动状态 *)
    s:REAL;                 (* 进度 0~1 *)
    sVel:REAL;
    sAcc:REAL;
    sStop:REAL;
    sRem:REAL;
    phase:INT;

    (* 临时变量 *)
    sx,sy,sz:REAL;          (* 起点 *)
    ex,ey,ez:REAL;          (* 终点 *)
    mx,my,mz:REAL;          (* 中间点(三点式) *)
    smx,smy,smz:REAL;       (* 起点→中间点 *)
    sex,sey,sez:REAL;       (* 起点→终点 *)
    nx,ny,nz:REAL;          (* 法向量 *)
    nLen:REAL;
    smLen:REAL;
    eu,ev:REAL;
    mu,mv:REAL;             (* 中间点在u-v平面坐标 *)
    aa,bb:REAL;
    csx,csy,csz:REAL;       (* 圆心→起点 *)
    cex,cey,cez:REAL;       (* 圆心→终点 *)
    cmx,cmy,cmz:REAL;       (* 圆心→中间点 *)
    thetaMid:REAL;          (* 中间点角度 *)
    theta:REAL;
    cosT,sinT:REAL;
    exLast:BOOL;
    exRise:BOOL;
    enLast:BOOL;
    initialized:BOOL;
END_VAR


IF NOT Enable THEN
    PlanX:=0.0; PlanY:=0.0; PlanZ:=0.0;
    PlanVX:=0.0; PlanVY:=0.0; PlanVZ:=0.0;
    InPos:=FALSE; Busy:=FALSE; phase:=0; s:=0.0;
    initialized:=FALSE;
    RETURN;
END_IF;

IF NOT enLast THEN
    PlanX := InitX; PlanY := InitY; PlanZ := InitZ;
END_IF;
enLast := Enable;

exRise := Execute AND NOT exLast;
exLast := Execute;


(* ============================================================================
   启动：求圆心、半径、平面基向量、夹角
   ============================================================================ *)
IF exRise THEN
    (* 首次运动用 Init（实际位置）作起点，后续用上一次规划终点 *)
    IF NOT initialized THEN
        sx := InitX; sy := InitY; sz := InitZ;
        initialized := TRUE;
    ELSE
        sx := PlanX; sy := PlanY; sz := PlanZ;
    END_IF;
    ex := TargetX; ey := TargetY; ez := TargetZ;

    IF CirMode = 0 THEN
        (* ---------- 圆心式 ---------- *)
        cx := AuxX; cy := AuxY; cz := AuxZ;
    ELSE
        (* ---------- 三点式：求外接圆圆心 ---------- *)
        mx := AuxX; my := AuxY; mz := AuxZ;
        smx := mx-sx; smy := my-sy; smz := mz-sz;
        sex := ex-sx; sey := ey-sy; sez := ez-sz;
        (* 法向量 n = SM × SE *)
        nx := smy*sez - smz*sey;
        ny := smz*sex - smx*sez;
        nz := smx*sey - smy*sex;
        nLen := SQRT(nx*nx+ny*ny+nz*nz);
        IF nLen < 1.0E-6 THEN
            (* 三点共线，退化为直线 *)
            phase := 0; InPos := TRUE; Busy := FALSE;
            RETURN;
        END_IF;
        nx := nx/nLen; ny := ny/nLen; nz := nz/nLen;
        smLen := SQRT(smx*smx+smy*smy+smz*smz);
        ux := smx/smLen; uy := smy/smLen; uz := smz/smLen;
        (* v = n × u *)
        vx := ny*uz - nz*uy;
        vy := nz*ux - nx*uz;
        vz := nx*uy - ny*ux;
        (* E 在 u-v 平面的坐标 *)
        eu := sex*ux + sey*uy + sez*uz;
        ev := sex*vx + sey*vy + sez*vz;
        aa := smLen / 2.0;
        IF ABS(ev) < 1.0E-6 THEN
            phase := 0; InPos := TRUE; Busy := FALSE;
            RETURN;
        END_IF;
        bb := (eu*eu + ev*ev - 2.0*aa*eu) / (2.0*ev);
        (* 圆心 C = S + aa·u + bb·v *)
        cx := sx + aa*ux + bb*vx;
        cy := sy + aa*uy + bb*vy;
        cz := sz + aa*uz + bb*vz;
    END_IF;

    (* ---------- 计算半径、基向量u、v ---------- *)
    csx := sx-cx; csy := sy-cy; csz := sz-cz;
    radius := SQRT(csx*csx+csy*csy+csz*csz);
    IF radius < 1.0E-6 THEN
        phase := 0; InPos := TRUE; Busy := FALSE;
        RETURN;
    END_IF;
    ux := csx/radius; uy := csy/radius; uz := csz/radius;

    (* 法向量：圆心式需要从起点和终点求 *)
    cex := ex-cx; cey := ey-cy; cez := ez-cz;
    IF CirMode = 0 THEN
        nx := csy*cez - csz*cey;
        ny := csz*cex - csx*cez;
        nz := csx*cey - csy*cex;
        nLen := SQRT(nx*nx+ny*ny+nz*nz);
        IF nLen < 1.0E-6 THEN
            phase := 0; InPos := TRUE; Busy := FALSE;
            RETURN;
        END_IF;
        nx := nx/nLen; ny := ny/nLen; nz := nz/nLen;
    END_IF;
    (* v = n × u *)
    vx := ny*uz - nz*uy;
    vy := nz*ux - nx*uz;
    vz := nx*uy - ny*ux;

    (* ---------- 计算夹角 ---------- *)
    (* 终点在 u-v 平面的坐标 *)
    eu := cex*ux + cey*uy + cez*uz;
    ev := cex*vx + cey*vy + cez*vz;
    thetaStart := 0.0;
    thetaEnd := ATAN2(ev, eu);   (* 范围 -π~π *)

    (* 判断方向：三点式用中间点判断，圆心式默认逆时针(正) *)
    IF CirMode = 1 THEN
        (* 中间点在 u-v 平面的角度，判断短弧方向 *)
        cmx := mx-cx; cmy := my-cy; cmz := mz-cz;
        mu := cmx*ux + cmy*uy + cmz*uz;
        mv := cmx*vx + cmy*vy + cmz*vz;
        thetaMid := ATAN2(mv, mu);
        (* 如果终点角为负，调整到正范围 *)
        IF thetaEnd < 0.0 THEN thetaEnd := thetaEnd + 6.283185307179586; END_IF;
        IF thetaMid < 0.0 THEN thetaMid := thetaMid + 6.283185307179586; END_IF;
        (* 若中间点不在 0→thetaEnd 的弧上，说明走长弧，thetaEnd 取负方向 *)
        IF NOT (thetaMid > 0.0 AND thetaMid < thetaEnd) THEN
            thetaEnd := thetaEnd - 6.283185307179586;
        END_IF;
    ELSE
        (* 圆心式：默认走短弧 *)
        IF thetaEnd < 0.0 THEN thetaEnd := thetaEnd + 6.283185307179586; END_IF;
        IF thetaEnd > 3.141592653589793 THEN thetaEnd := thetaEnd - 6.283185307179586; END_IF;
    END_IF;

    thetaSpan := thetaEnd;
    arcLen := radius * ABS(thetaSpan);

    IF arcLen < 1.0E-4 THEN
        PlanX:=ex; PlanY:=ey; PlanZ:=ez;
        PlanVX:=0.0; PlanVY:=0.0; PlanVZ:=0.0;
        s:=1.0; phase:=4; InPos:=TRUE; Busy:=FALSE;
    ELSE
        (* 转换到进度域：v_s = v_cart / arcLen *)
        maxSVel := Speed / arcLen;
        maxSAcc  := Accel / arcLen;
        maxSJerk := Jerk / arcLen;
        s:=0.0; sVel:=0.0; sAcc:=0.0; phase:=1;
        InPos:=FALSE; Busy:=TRUE;
    END_IF;
END_IF;


(* ============================================================================
   每周期：加减速 + 圆弧插补
   ============================================================================ *)
IF phase <> 4 AND phase <> 0 THEN

    sRem := 1.0 - s;

    (* 减速距离估计（进度域） *)
    IF MoveMode = 0 THEN
        sStop := sVel*sVel / (2.0*maxSAcc);
    ELSE
        sStop := sVel*sVel/(2.0*maxSAcc) + sVel*maxSAcc/(2.0*maxSJerk);
    END_IF;

    CASE phase OF
        1:
            IF sRem <= sStop THEN
                phase := 3;
            ELSE
                IF MoveMode = 0 THEN
                    sAcc := maxSAcc;
                ELSE
                    sAcc := sAcc + maxSJerk*CycleTime;
                    IF sAcc > maxSAcc THEN sAcc := maxSAcc; END_IF;
                END_IF;
                sVel := sVel + sAcc*CycleTime;
                IF sVel >= maxSVel THEN
                    sVel := maxSVel; sAcc := 0.0; phase := 2;
                END_IF;
            END_IF;
        2:
            IF sRem <= sStop THEN phase := 3;
            ELSE sVel := maxSVel; sAcc := 0.0;
            END_IF;
        3:
            IF MoveMode = 0 THEN
                sAcc := -maxSAcc;
            ELSE
                sAcc := sAcc - maxSJerk*CycleTime;
                IF sAcc < -maxSAcc THEN sAcc := -maxSAcc; END_IF;
            END_IF;
            sVel := sVel + sAcc*CycleTime;
            IF sVel <= 0.0 THEN
                sVel := 0.0; sAcc := 0.0;
                s := 1.0; phase := 4;
            END_IF;
    END_CASE;

    IF phase <> 4 THEN
        s := s + sVel*CycleTime;
        IF s > 1.0 THEN s := 1.0; END_IF;
    END_IF;

    (* 圆弧插补：theta = thetaStart + s × thetaSpan *)
    theta := thetaStart + s * thetaSpan;
    cosT := COS(theta); sinT := SIN(theta);
    (* 点 = 圆心 + R×(cosθ·u + sinθ·v) *)
    PlanX := cx + radius*(cosT*ux + sinT*vx);
    PlanY := cy + radius*(cosT*uy + sinT*vy);
    PlanZ := cz + radius*(cosT*uz + sinT*vz);

    (* 规划速度：v = R × θ̇ × (-sinθ·u + cosθ·v)，θ̇ = sVel × thetaSpan *)
    PlanVX := radius * (sVel*thetaSpan) * (-sinT*ux + cosT*vx);
    PlanVY := radius * (sVel*thetaSpan) * (-sinT*uy + cosT*vy);
    PlanVZ := radius * (sVel*thetaSpan) * (-sinT*uz + cosT*vz);

    IF phase = 4 THEN
        PlanX := ex; PlanY := ey; PlanZ := ez;
        PlanVX := 0.0; PlanVY := 0.0; PlanVZ := 0.0;
        InPos := TRUE; Busy := FALSE;
    END_IF;
END_IF;

END_FUNCTION_BLOCK
