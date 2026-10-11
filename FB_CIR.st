(* ============================================================================
   功能块名称 : FB_CIR
   功能描述 : Delta 机器人笛卡尔空间圆弧插补指令 FB（PLCopen 风格）
              支持两种模式：
                CirMode=0 圆心式：起点(Axis.Cur) + 终点(Position) + 圆心(Aux)
                CirMode=1 三点式：起点(Axis.Cur) + 中间点(Aux) + 终点(Position)
              按弧长均匀插补，加减速在弧长方向
   调用方式 : 每周期先调用各指令FB，最后调用 FB_DeltaCore
              Execute 上升沿启动；多指令FB共享同一 Axis(VAR_IN_OUT) 协调
   BufferMode: 0=拒绝 1=打断 2=缓存
   ============================================================================ *)

FUNCTION_BLOCK FB_CIR

VAR_INPUT
    Execute      : BOOL;    (* 启动（上升沿有效）                       *)
    PositionX    : REAL;    (* 终点位置 X (mm)                          *)
    PositionY    : REAL;
    PositionZ    : REAL;
    AuxX         : REAL;    (* 辅助点 X：圆心式=圆心，三点式=中间点      *)
    AuxY         : REAL;
    AuxZ         : REAL;
    CirMode      : INT;     (* 0=圆心式  1=三点式                       *)
    Speed        : REAL;    (* 最大速度 (mm/s)                          *)
    Accel        : REAL;    (* 最大加速度 (mm/s²)                       *)
    Jerk         : REAL;    (* 最大加加速度 (mm/s³)，S曲线用            *)
    MoveMode     : INT;     (* 0=梯形 1=S曲线                           *)
    BufferMode   : INT;     (* 0=拒绝 1=打断 2=缓存                     *)
END_VAR

VAR_IN_OUT
    Axis         : st_DeltaAxis;   (* 共享轴状态                        *)
END_VAR

VAR_OUTPUT
    Busy            : BOOL;
    InPos           : BOOL;
    CommandAborted  : BOOL;
    Rejected        : BOOL;
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
    s:REAL;
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
    aa,bb:REAL;
    cex,cey,cez:REAL;       (* 圆心→终点 *)
    cmx,cmy,cmz:REAL;       (* 圆心→中间点 *)
    thetaMid:REAL;          (* 中间点角度 *)
    theta:REAL;
    cosT,sinT:REAL;
    geomFail:BOOL;          (* 几何退化（共线/零半径）标志 *)

    (* 调度状态 *)
    myId:DINT;
    running:BOOL;
    waiting:BOOL;
    wantStart:BOOL;
    exLast:BOOL;
    exRise:BOOL;
END_VAR


(* 失能：复位自身，不写 Axis 规划 *)
IF NOT Axis.Enable THEN
    Busy:=FALSE; InPos:=FALSE; CommandAborted:=FALSE; Rejected:=FALSE;
    running:=FALSE; waiting:=FALSE; phase:=0; s:=0.0;
    exLast:=Execute;
    RETURN;
END_IF;

Rejected:=FALSE;
exRise := Execute AND NOT exLast;
exLast := Execute;
wantStart := FALSE;


(* 第一步：上升沿仲裁 *)
IF exRise THEN
    InPos:=FALSE; CommandAborted:=FALSE; waiting:=FALSE;

    IF (Speed <= 0.0) OR (Accel <= 0.0) OR ((MoveMode = 1) AND (Jerk <= 0.0)) THEN
        Rejected:=TRUE;
    ELSIF running AND (Axis.owner = myId) THEN
        wantStart:=TRUE;
    ELSIF Axis.Busy THEN
        CASE BufferMode OF
            0: Rejected:=TRUE;
            1: wantStart:=TRUE;
            2:
                IF Axis.hasBuf THEN
                    Rejected:=TRUE;
                ELSE
                    waiting:=TRUE; Axis.hasBuf:=TRUE; Busy:=TRUE;
                END_IF;
        END_CASE;
    ELSE
        wantStart:=TRUE;
    END_IF;
END_IF;

(* 第二步：缓存等待处理 *)
IF waiting THEN
    IF NOT Axis.hasBuf THEN
        waiting:=FALSE; Busy:=FALSE; CommandAborted:=TRUE;
    ELSIF NOT Axis.Busy THEN
        waiting:=FALSE; Axis.hasBuf:=FALSE; wantStart:=TRUE;
    END_IF;
END_IF;

(* 第三步：统一启动（令牌 + 几何求解） *)
IF wantStart THEN
    myId:=Axis.cmdCount + 1;
    Axis.cmdCount:=myId;
    Axis.owner:=myId;
    Axis.ActiveType:=3;
    Axis.Busy:=TRUE;
    Axis.InPos:=FALSE;
    Axis.hasBuf:=FALSE;
    running:=TRUE; Busy:=TRUE; InPos:=FALSE;
    geomFail:=FALSE;

    (* 以核心层 FK 实际位置为起点，保证与实际位置连续 *)
    sx := Axis.CurX; sy := Axis.CurY; sz := Axis.CurZ;
    ex := PositionX; ey := PositionY; ez := PositionZ;

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
            geomFail:=TRUE;
        ELSE
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
                geomFail:=TRUE;
            ELSE
                bb := (eu*eu + ev*ev - 2.0*aa*eu) / (2.0*ev);
                (* 圆心 C = S + aa·u + bb·v *)
                cx := sx + aa*ux + bb*vx;
                cy := sy + aa*uy + bb*vy;
                cz := sz + aa*uz + bb*vz;
            END_IF;
        END_IF;
    END_IF;

    IF NOT geomFail THEN
        (* ---------- 计算半径、基向量u、v ---------- *)
        csx := sx-cx; csy := sy-cy; csz := sz-cz;
        radius := SQRT(csx*csx+csy*csy+csz*csz);
        cex := ex-cx; cey := ey-cy; cez := ez-cz;
        IF radius < 1.0E-6 THEN
            geomFail:=TRUE;
        ELSE
            ux := csx/radius; uy := csy/radius; uz := csz/radius;

            (* 法向量：圆心式需要从起点和终点求 *)
            IF CirMode = 0 THEN
                nx := csy*cez - csz*cey;
                ny := csz*cex - csx*cez;
                nz := csx*cey - csy*cex;
                nLen := SQRT(nx*nx+ny*ny+nz*nz);
                IF nLen < 1.0E-6 THEN
                    geomFail:=TRUE;
                ELSE
                    nx := nx/nLen; ny := ny/nLen; nz := nz/nLen;
                END_IF;
            END_IF;
        END_IF;
    END_IF;

    IF NOT geomFail THEN
        (* v = n × u *)
        vx := ny*uz - nz*uy;
        vy := nz*ux - nx*uz;
        vz := nx*uy - ny*ux;

        (* ---------- 计算夹角 ---------- *)
        eu := cex*ux + cey*uy + cez*uz;
        ev := cex*vx + cey*vy + cez*vz;
        thetaStart := 0.0;
        thetaEnd := ATAN2(ev, eu);   (* 范围 -π~π *)

        (* 判断方向：三点式用中间点判断，圆心式走短弧 *)
        IF CirMode = 1 THEN
            cmx := mx-cx; cmy := my-cy; cmz := mz-cz;
            thetaMid := ATAN2(cmx*vx+cmy*vy+cmz*vz, cmx*ux+cmy*uy+cmz*uz);
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
    END_IF;

    IF geomFail THEN
        (* 几何退化（三点共线/零半径等）：立即释放轴，信号按完成处理 *)
        running:=FALSE; Busy:=FALSE; InPos:=TRUE;
        Axis.Busy:=FALSE; Axis.ActiveType:=0; Axis.owner:=0; Axis.InPos:=TRUE;
    ELSIF arcLen < 1.0E-4 THEN
        (* 零弧长：当周期完成 *)
        Axis.PlanX:=ex; Axis.PlanY:=ey; Axis.PlanZ:=ez;
        Axis.PlanVX:=0.0; Axis.PlanVY:=0.0; Axis.PlanVZ:=0.0;
        s:=1.0; phase:=4;
    ELSE
        (* 转换到进度域：v_s = v_cart / arcLen *)
        maxSVel := Speed / arcLen;
        maxSAcc  := Accel / arcLen;
        maxSJerk := Jerk / arcLen;
        s:=0.0; sVel:=0.0; sAcc:=0.0; phase:=1;
    END_IF;
END_IF;

(* 第四步：被其他指令打断 *)
IF running AND (Axis.owner <> myId) THEN
    running:=FALSE; phase:=0; Busy:=FALSE; CommandAborted:=TRUE;
END_IF;

(* 第五步：加减速 + 圆弧插补 → 写 Axis.Plan* *)
IF running AND (phase = 1 OR phase = 2 OR phase = 3) THEN

    sRem := 1.0 - s;

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
                    sAcc := sAcc + maxSJerk*Axis.CycleTime;
                    IF sAcc > maxSAcc THEN sAcc := maxSAcc; END_IF;
                END_IF;
                sVel := sVel + sAcc*Axis.CycleTime;
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
                sAcc := sAcc - maxSJerk*Axis.CycleTime;
                IF sAcc < -maxSAcc THEN sAcc := -maxSAcc; END_IF;
            END_IF;
            sVel := sVel + sAcc*Axis.CycleTime;
            IF sVel <= 0.0 THEN
                sVel := 0.0; sAcc := 0.0;
                s := 1.0; phase := 4;
            END_IF;
    END_CASE;

    IF phase <> 4 THEN
        s := s + sVel*Axis.CycleTime;
        IF s > 1.0 THEN s := 1.0; END_IF;
    END_IF;

    (* 圆弧插补：theta = thetaStart + s × thetaSpan *)
    theta := thetaStart + s * thetaSpan;
    cosT := COS(theta); sinT := SIN(theta);
    (* 点 = 圆心 + R×(cosθ·u + sinθ·v) *)
    Axis.PlanX := cx + radius*(cosT*ux + sinT*vx);
    Axis.PlanY := cy + radius*(cosT*uy + sinT*vy);
    Axis.PlanZ := cz + radius*(cosT*uz + sinT*vz);

    (* 规划速度：v = R × θ̇ × (-sinθ·u + cosθ·v)，θ̇ = sVel × thetaSpan *)
    Axis.PlanVX := radius * (sVel*thetaSpan) * (-sinT*ux + cosT*vx);
    Axis.PlanVY := radius * (sVel*thetaSpan) * (-sinT*uy + cosT*vy);
    Axis.PlanVZ := radius * (sVel*thetaSpan) * (-sinT*uz + cosT*vz);

    IF phase = 4 THEN
        Axis.PlanX := ex; Axis.PlanY := ey; Axis.PlanZ := ez;
        Axis.PlanVX := 0.0; Axis.PlanVY := 0.0; Axis.PlanVZ := 0.0;
    END_IF;
END_IF;

(* 第六步：完成 → 释放轴控制权 *)
IF running AND (phase = 4) THEN
    running:=FALSE;
    Busy:=FALSE; InPos:=TRUE;
    Axis.Busy:=FALSE; Axis.ActiveType:=0; Axis.owner:=0; Axis.InPos:=TRUE;
END_IF;

END_FUNCTION_BLOCK
