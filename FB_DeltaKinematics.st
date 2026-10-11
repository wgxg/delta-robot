(* ============================================================================
   功能块名称 : FB_DeltaKinematics  （基础层）
   功能描述 : Delta 机器人纯运动学转换（不含位置规划）
              1. FK：当前角度 → 吸盘当前位置
              2. IK：规划位置 → 目标角度
              3. 速度前馈：补偿高速位置环滞后
   调用方式 : 周期调用，由上层运动规划FB提供规划位置
   坐标系   : 原点在静平台中心，Z 轴向下，工作空间 Z > 0
   角度单位 : rad     长度单位 : mm     速度 : mm/s
   ----------------------------------------------------------------------------
   版本     : V4.0  拆分为基础层，去掉位置规划
   ============================================================================ *)

FUNCTION_BLOCK FB_DeltaKinematics

VAR_INPUT
    (* ===================== 机械结构参数 ===================== *)
    R          : REAL;      (* 静平台铰点分布圆半径 (mm)            *)
    r          : REAL;      (* 动平台铰点分布圆半径 (mm)            *)
    L          : REAL;      (* 主动臂长度 (mm)                      *)
    l          : REAL;      (* 从动臂长度 (mm)                      *)

    (* ===================== 规划位置（来自运动规划FB） ===================== *)
    PlanX      : REAL;      (* 规划位置 X (mm)                      *)
    PlanY      : REAL;      (* 规划位置 Y (mm)                      *)
    PlanZ      : REAL;      (* 规划位置 Z (mm)                      *)

    (* ===================== 规划速度（前馈用） ===================== *)
    PlanVX     : REAL;      (* 规划速度 X (mm/s)                    *)
    PlanVY     : REAL;      (* 规划速度 Y (mm/s)                    *)
    PlanVZ     : REAL;      (* 规划速度 Z (mm/s)                    *)

    (* ===================== 前馈参数 ===================== *)
    FfEnable   : BOOL;      (* 速度前馈使能                          *)
    Kff        : REAL;      (* 前馈增益 (0~1)                        *)

    (* ===================== 反馈与时序 ===================== *)
    CurTheta1  : REAL;      (* 第 1 轴当前角度 (rad)                *)
    CurTheta2  : REAL;      (* 第 2 轴当前角度 (rad)                *)
    CurTheta3  : REAL;      (* 第 3 轴当前角度 (rad)                *)
    CycleTime  : REAL;      (* 调用周期 (s)                         *)

    (* ===================== 使能 ===================== *)
    Enable     : BOOL;      (* 使能                                 *)
END_VAR

VAR_OUTPUT
    TarTheta1  : REAL;      (* 第 1 轴目标角度 (rad，含前馈)        *)
    TarTheta2  : REAL;      (* 第 2 轴目标角度                      *)
    TarTheta3  : REAL;      (* 第 3 轴目标角度                      *)
    CurX       : REAL;      (* 当前位置 X (FK 结果)                 *)
    CurY       : REAL;      (* 当前位置 Y                           *)
    CurZ       : REAL;      (* 当前位置 Z                           *)
    IK_OK      : BOOL;      (* 逆运动学成功标志                     *)
    FK_OK      : BOOL;      (* 正运动学成功标志                     *)
END_VAR

VAR
    COS1 : REAL := 1.0;
    SIN1 : REAL := 0.0;
    COS2 : REAL := -0.5;
    SIN2 : REAL :=  0.8660254037844386;
    COS3 : REAL := -0.5;
    SIN3 : REAL := -0.8660254037844386;

    xp, yp        : REAL;
    A, B, C       : REAL;
    Disc          : REAL;
    u1, u2        : REAL;
    thetaSol1     : REAL;
    thetaSol2     : REAL;
    denom         : REAL;
    s1, s2, s3    : REAL;
    sn1, sn2, sn3 : REAL;
    a1, b1, c1, d1: REAL;
    a2, b2, c2, d2: REAL;
    Det           : REAL;
    px, py        : REAL;
    qx, qy        : REAL;
    X0, Y0        : REAL;
    Az, Bz, Cz    : REAL;
    Dz            : REAL;
    z1, z2        : REAL;
    e             : REAL;
    Aii           : REAL;
    Bix, Biy, Biz : REAL;
    dTh1_ff, dTh2_ff, dTh3_ff : REAL;
    ikTh1, ikTh2, ikTh3 : REAL;
    ct, st        : REAL;
END_VAR


IF NOT Enable THEN
    TarTheta1 := 0.0; TarTheta2 := 0.0; TarTheta3 := 0.0;
    CurX := 0.0; CurY := 0.0; CurZ := 0.0;
    IK_OK := FALSE; FK_OK := FALSE;
    RETURN;
END_IF;

e := R - r;


(* ============================================================================
   【FK】当前角度 → 当前位置
   ============================================================================ *)
s1 := e + L * COS(CurTheta1);
s2 := e + L * COS(CurTheta2);
s3 := e + L * COS(CurTheta3);
sn1 := L * SIN(CurTheta1);
sn2 := L * SIN(CurTheta2);
sn3 := L * SIN(CurTheta3);

a1 := -2.0 * (s2 * COS2 - s1 * COS1);
b1 := -2.0 * (s2 * SIN2 - s1 * SIN1);
c1 := -2.0 * (sn2 - sn1);
d1 := (s1*s1 + sn1*sn1) - (s2*s2 + sn2*sn2);

a2 := -2.0 * (s3 * COS3 - s1 * COS1);
b2 := -2.0 * (s3 * SIN3 - s1 * SIN1);
c2 := -2.0 * (sn3 - sn1);
d2 := (s1*s1 + sn1*sn1) - (s3*s3 + sn3*sn3);

Det := a1 * b2 - a2 * b1;

IF ABS(Det) < 1.0E-9 THEN
    FK_OK := FALSE; CurX := 0.0; CurY := 0.0; CurZ := 0.0;
ELSE
    px := (b2 * d1 - b1 * d2) / Det;
    qx := (b1 * c2 - b2 * c1) / Det;
    py := (a1 * d2 - a2 * d1) / Det;
    qy := (a2 * c1 - a1 * c2) / Det;
    X0 := px - s1 * COS1;
    Y0 := py - s1 * SIN1;
    Az := qx*qx + qy*qy + 1.0;
    Bz := 2.0 * (X0*qx + Y0*qy) - 2.0 * sn1;
    Cz := X0*X0 + Y0*Y0 + sn1*sn1 - l*l;
    Dz := Bz*Bz - 4.0*Az*Cz;

    IF Dz < 0.0 THEN
        FK_OK := FALSE; CurX := 0.0; CurY := 0.0; CurZ := 0.0;
    ELSE
        z1 := (-Bz + SQRT(Dz)) / (2.0 * Az);
        z2 := (-Bz - SQRT(Dz)) / (2.0 * Az);
        IF z1 > 0.0 THEN CurZ := z1;
        ELSIF z2 > 0.0 THEN CurZ := z2;
        ELSE CurZ := z1;
        END_IF;
        CurX := px + qx * CurZ;
        CurY := py + qy * CurZ;
        FK_OK := TRUE;
    END_IF;
END_IF;


(* ============================================================================
   【IK】规划位置 → 目标角度
   ============================================================================ *)

(* ---------- 第 1 轴 ---------- *)
xp := PlanX * COS1 + PlanY * SIN1;
yp := -PlanX * SIN1 + PlanY * COS1;
A := 2.0 * L * (e - xp);
B := -2.0 * L * PlanZ;
C := L*L + (e-xp)*(e-xp) + yp*yp + PlanZ*PlanZ - l*l;
Disc := A*A + B*B - C*C;

IF Disc < 0.0 THEN
    IK_OK := FALSE; ikTh1 := TarTheta1;
ELSE
    denom := C - A;
    IF ABS(denom) < 1.0E-9 THEN
        IK_OK := FALSE; ikTh1 := TarTheta1;
    ELSE
        u1 := (-B + SQRT(Disc)) / denom;
        u2 := (-B - SQRT(Disc)) / denom;
        thetaSol1 := 2.0 * ATAN(u1);
        thetaSol2 := 2.0 * ATAN(u2);
        IF thetaSol1 > 0.0 AND thetaSol2 > 0.0 THEN
            IF ABS(thetaSol1 - CurTheta1) <= ABS(thetaSol2 - CurTheta1) THEN
                ikTh1 := thetaSol1; ELSE ikTh1 := thetaSol2;
            END_IF;
        ELSIF thetaSol1 > 0.0 THEN ikTh1 := thetaSol1;
        ELSIF thetaSol2 > 0.0 THEN ikTh1 := thetaSol2;
        ELSE ikTh1 := thetaSol1;
        END_IF;
        IK_OK := TRUE;
    END_IF;
END_IF;

(* ---------- 第 2 轴 ---------- *)
xp := PlanX * COS2 + PlanY * SIN2;
yp := -PlanX * SIN2 + PlanY * COS2;
A := 2.0 * L * (e - xp);
B := -2.0 * L * PlanZ;
C := L*L + (e-xp)*(e-xp) + yp*yp + PlanZ*PlanZ - l*l;
Disc := A*A + B*B - C*C;

IF Disc < 0.0 THEN
    IK_OK := FALSE; ikTh2 := TarTheta2;
ELSE
    denom := C - A;
    IF ABS(denom) < 1.0E-9 THEN
        IK_OK := FALSE; ikTh2 := TarTheta2;
    ELSE
        u1 := (-B + SQRT(Disc)) / denom;
        u2 := (-B - SQRT(Disc)) / denom;
        thetaSol1 := 2.0 * ATAN(u1);
        thetaSol2 := 2.0 * ATAN(u2);
        IF thetaSol1 > 0.0 AND thetaSol2 > 0.0 THEN
            IF ABS(thetaSol1 - CurTheta2) <= ABS(thetaSol2 - CurTheta2) THEN
                ikTh2 := thetaSol1; ELSE ikTh2 := thetaSol2;
            END_IF;
        ELSIF thetaSol1 > 0.0 THEN ikTh2 := thetaSol1;
        ELSIF thetaSol2 > 0.0 THEN ikTh2 := thetaSol2;
        ELSE ikTh2 := thetaSol1;
        END_IF;
    END_IF;
END_IF;

(* ---------- 第 3 轴 ---------- *)
xp := PlanX * COS3 + PlanY * SIN3;
yp := -PlanX * SIN3 + PlanY * COS3;
A := 2.0 * L * (e - xp);
B := -2.0 * L * PlanZ;
C := L*L + (e-xp)*(e-xp) + yp*yp + PlanZ*PlanZ - l*l;
Disc := A*A + B*B - C*C;

IF Disc < 0.0 THEN
    IK_OK := FALSE; ikTh3 := TarTheta3;
ELSE
    denom := C - A;
    IF ABS(denom) < 1.0E-9 THEN
        IK_OK := FALSE; ikTh3 := TarTheta3;
    ELSE
        u1 := (-B + SQRT(Disc)) / denom;
        u2 := (-B - SQRT(Disc)) / denom;
        thetaSol1 := 2.0 * ATAN(u1);
        thetaSol2 := 2.0 * ATAN(u2);
        IF thetaSol1 > 0.0 AND thetaSol2 > 0.0 THEN
            IF ABS(thetaSol1 - CurTheta3) <= ABS(thetaSol2 - CurTheta3) THEN
                ikTh3 := thetaSol1; ELSE ikTh3 := thetaSol2;
            END_IF;
        ELSIF thetaSol1 > 0.0 THEN ikTh3 := thetaSol1;
        ELSIF thetaSol2 > 0.0 THEN ikTh3 := thetaSol2;
        ELSE ikTh3 := thetaSol1;
        END_IF;
    END_IF;
END_IF;


(* ============================================================================
   【速度前馈】Δθ_ff = A⁻¹·B·v_plan · CycleTime
   ============================================================================ *)
dTh1_ff := 0.0; dTh2_ff := 0.0; dTh3_ff := 0.0;

IF FfEnable THEN
    xp := PlanX * COS1 + PlanY * SIN1;
    yp := -PlanX * SIN1 + PlanY * COS1;
    ct := COS(ikTh1); st := SIN(ikTh1);
    Aii := -2.0 * L * ((e - xp) * st + PlanZ * ct);
    Bix := -2.0 * ((L*ct + e - xp) * COS1 + yp * SIN1);
    Biy := -2.0 * ((L*ct + e - xp) * SIN1 - yp * COS1);
    Biz := -2.0 * (L * st - PlanZ);
    IF ABS(Aii) > 1.0E-9 THEN
        dTh1_ff := ((Bix*PlanVX + Biy*PlanVY + Biz*PlanVZ) / Aii) * CycleTime;
    END_IF;

    xp := PlanX * COS2 + PlanY * SIN2;
    yp := -PlanX * SIN2 + PlanY * COS2;
    ct := COS(ikTh2); st := SIN(ikTh2);
    Aii := -2.0 * L * ((e - xp) * st + PlanZ * ct);
    Bix := -2.0 * ((L*ct + e - xp) * COS2 + yp * SIN2);
    Biy := -2.0 * ((L*ct + e - xp) * SIN2 - yp * COS2);
    Biz := -2.0 * (L * st - PlanZ);
    IF ABS(Aii) > 1.0E-9 THEN
        dTh2_ff := ((Bix*PlanVX + Biy*PlanVY + Biz*PlanVZ) / Aii) * CycleTime;
    END_IF;

    xp := PlanX * COS3 + PlanY * SIN3;
    yp := -PlanX * SIN3 + PlanY * COS3;
    ct := COS(ikTh3); st := SIN(ikTh3);
    Aii := -2.0 * L * ((e - xp) * st + PlanZ * ct);
    Bix := -2.0 * ((L*ct + e - xp) * COS3 + yp * SIN3);
    Biy := -2.0 * ((L*ct + e - xp) * SIN3 - yp * COS3);
    Biz := -2.0 * (L * st - PlanZ);
    IF ABS(Aii) > 1.0E-9 THEN
        dTh3_ff := ((Bix*PlanVX + Biy*PlanVY + Biz*PlanVZ) / Aii) * CycleTime;
    END_IF;
END_IF;

TarTheta1 := ikTh1 + Kff * dTh1_ff;
TarTheta2 := ikTh2 + Kff * dTh2_ff;
TarTheta3 := ikTh3 + Kff * dTh3_ff;

END_FUNCTION_BLOCK
