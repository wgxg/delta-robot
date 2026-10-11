(* ============================================================================
   功能块名称 : FB_LIN
   功能描述 : Delta 机器人笛卡尔空间直线插补
              Plan = Start + dir × (s × totalDist)
              s 按加减速曲线（梯形/S曲线）从 0 走到 1
              起点为上一次的规划位置（首次使用需外部初始化）
   调用方式 : 周期调用，Execute 上升沿启动
   ============================================================================ *)

FUNCTION_BLOCK FB_LIN

VAR_INPUT
    TargetX      : REAL;    (* 目标位置 X (mm)                       *)
    TargetY      : REAL;
    TargetZ      : REAL;
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
    startX,startY,startZ:REAL;  (* 起点 *)
    tgtX,tgtY,tgtZ:REAL;        (* 目标位置(启动时存入) *)
    dirX,dirY,dirZ:REAL;        (* 方向单位向量 *)
    totalDist:REAL;             (* 总距离 *)
    maxSVel:REAL;               (* 进度域最大速度 = Speed/totalDist (1/s) *)
    maxSAcc:REAL;               (* 进度域最大加速度 (1/s²) *)
    maxSJerk:REAL;              (* 进度域最大加加速度 (1/s³) *)
    s:REAL;                     (* 进度 0~1 *)
    sVel:REAL;                  (* 进度速度 (1/s) *)
    sAcc:REAL;                  (* 进度加速度 (1/s²) *)
    sStop:REAL;                 (* 减速距离（进度域） *)
    sRem:REAL;                  (* 剩余距离（进度域） *)
    phase:INT;                  (* 0=idle 1=accel 2=const 3=decel 4=done *)
    exLast:BOOL;
    exRise:BOOL;
    enLast:BOOL;
END_VAR


IF NOT Enable THEN
    PlanX:=0.0; PlanY:=0.0; PlanZ:=0.0;
    PlanVX:=0.0; PlanVY:=0.0; PlanVZ:=0.0;
    InPos:=FALSE; Busy:=FALSE; phase:=0; s:=0.0;
    RETURN;
END_IF;

(* 首次启用：用 InitX/Y/Z 初始化规划位置（仅用于显示，不影响起点判断） *)
IF NOT enLast THEN
    PlanX := InitX; PlanY := InitY; PlanZ := InitZ;
END_IF;
enLast := Enable;

(* Execute 上升沿 *)
exRise := Execute AND NOT exLast;
exLast := Execute;


(* ============================================================================
   启动：记录起点，计算方向和总距离
   ============================================================================ *)
IF exRise THEN
    (* 始终以 Init（核心层提供的FK实际位置）为起点，保证与实际位置连续 *)
    startX := InitX; startY := InitY; startZ := InitZ;
    dirX := TargetX - startX;
    dirY := TargetY - startY;
    dirZ := TargetZ - startZ;
    tgtX := TargetX; tgtY := TargetY; tgtZ := TargetZ;
    totalDist := SQRT(dirX*dirX + dirY*dirY + dirZ*dirZ);

    IF totalDist < 1.0E-4 THEN
        (* 已在目标位置 *)
        PlanX:=tgtX; PlanY:=tgtY; PlanZ:=tgtZ;
        PlanVX:=0.0; PlanVY:=0.0; PlanVZ:=0.0;
        s:=1.0; phase:=4; InPos:=TRUE; Busy:=FALSE;
    ELSE
        dirX := dirX / totalDist;
        dirY := dirY / totalDist;
        dirZ := dirZ / totalDist;
        (* 转换到进度域：v_s = v_cart / totalDist *)
        maxSVel := Speed / totalDist;
        maxSAcc  := Accel / totalDist;
        maxSJerk := Jerk / totalDist;
        s:=0.0; sVel:=0.0; sAcc:=0.0; phase:=1;
        InPos:=FALSE; Busy:=TRUE;
    END_IF;
END_IF;


(* ============================================================================
   每周期：加减速 + 直线插补
   ============================================================================ *)
IF phase <> 4 AND phase <> 0 THEN

    sRem := 1.0 - s;

    (* 减速距离估计（进度域） *)
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

    (* 直线插补：Plan = Start + dir × (s × totalDist) *)
    PlanX := startX + dirX * s * totalDist;
    PlanY := startY + dirY * s * totalDist;
    PlanZ := startZ + dirZ * s * totalDist;

    (* 规划速度 = dir × sVel × totalDist（进度速度转笛卡尔速度） *)
    PlanVX := dirX * sVel * totalDist;
    PlanVY := dirY * sVel * totalDist;
    PlanVZ := dirZ * sVel * totalDist;

    IF phase = 4 THEN
        PlanX := tgtX; PlanY := tgtY; PlanZ := tgtZ;
        PlanVX := 0.0; PlanVY := 0.0; PlanVZ := 0.0;
        InPos := TRUE; Busy := FALSE;
    END_IF;
END_IF;

END_FUNCTION_BLOCK
