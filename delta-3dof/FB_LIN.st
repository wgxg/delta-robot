(* ============================================================================
   功能块名称 : FB_LIN
   功能描述 : Delta 机器人笛卡尔空间直线插补指令 FB（PLCopen 风格）
              Plan = Start + dir × (s × totalDist)
              s 按加减速曲线（梯形/S曲线）从 0 走到 1
              起点取共享轴 Axis.CurX/Y/Z（核心层FK实际位置），保证连续
   调用方式 : 每周期先调用各指令FB，最后调用 FB_DeltaCore
              Execute 上升沿启动；多指令FB共享同一 Axis(VAR_IN_OUT) 协调
   BufferMode: 0=拒绝 1=打断 2=缓存
   ============================================================================ *)

FUNCTION_BLOCK FB_LIN

VAR_INPUT
    Execute      : BOOL;    (* 启动（上升沿有效）                       *)
    PositionX    : REAL;    (* 目标位置 X (mm)                          *)
    PositionY    : REAL;
    PositionZ    : REAL;
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

    (* 调度状态 *)
    myId:DINT;
    running:BOOL;
    waiting:BOOL;
    wantStart:BOOL;
    wasWaiting:BOOL;        (* 上升沿前本实例是否在缓存槽等待 *)
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
    (* 若本实例正在缓存槽等待：旧待执行指令作废并释放槽位，再按新指令仲裁；
       新指令被拒绝/槽满时，旧指令以 CommandAborted 留痕 *)
    wasWaiting := waiting;
    IF waiting THEN
        Axis.hasBuf:=FALSE;
        waiting:=FALSE; Busy:=FALSE;
    END_IF;

    InPos:=FALSE; CommandAborted:=FALSE;

    IF (Speed <= 0.0) OR (Accel <= 0.0) OR ((MoveMode = 1) AND (Jerk <= 0.0)) THEN
        Rejected:=TRUE;
        IF wasWaiting THEN CommandAborted:=TRUE; END_IF;
    ELSIF running AND (Axis.owner = myId) THEN
        wantStart:=TRUE;
    ELSIF Axis.Busy THEN
        CASE BufferMode OF
            0:
                Rejected:=TRUE;
                IF wasWaiting THEN CommandAborted:=TRUE; END_IF;
            1:
                wantStart:=TRUE;
            2:
                IF Axis.hasBuf THEN
                    Rejected:=TRUE;
                    IF wasWaiting THEN CommandAborted:=TRUE; END_IF;
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

(* 第三步：统一启动（令牌 + 几何初始化） *)
IF wantStart THEN
    myId:=Axis.cmdCount + 1;
    Axis.cmdCount:=myId;
    Axis.owner:=myId;
    Axis.ActiveType:=2;
    Axis.Busy:=TRUE;
    Axis.InPos:=FALSE;
    Axis.hasBuf:=FALSE;
    running:=TRUE; Busy:=TRUE; InPos:=FALSE;

    (* 以核心层 FK 实际位置为起点，保证与实际位置连续 *)
    startX := Axis.CurX; startY := Axis.CurY; startZ := Axis.CurZ;
    dirX := PositionX - startX;
    dirY := PositionY - startY;
    dirZ := PositionZ - startZ;
    tgtX := PositionX; tgtY := PositionY; tgtZ := PositionZ;
    totalDist := SQRT(dirX*dirX + dirY*dirY + dirZ*dirZ);

    IF totalDist < 1.0E-4 THEN
        Axis.PlanX:=tgtX; Axis.PlanY:=tgtY; Axis.PlanZ:=tgtZ;
        Axis.PlanVX:=0.0; Axis.PlanVY:=0.0; Axis.PlanVZ:=0.0;
        s:=1.0; phase:=4;
    ELSE
        dirX := dirX / totalDist;
        dirY := dirY / totalDist;
        dirZ := dirZ / totalDist;
        maxSVel := Speed / totalDist;
        maxSAcc  := Accel / totalDist;
        maxSJerk := Jerk / totalDist;
        s:=0.0; sVel:=0.0; sAcc:=0.0; phase:=1;
    END_IF;
END_IF;

(* 第四步：被其他指令打断 *)
IF running AND (Axis.owner <> myId) THEN
    running:=FALSE; phase:=0; Busy:=FALSE; CommandAborted:=TRUE;
END_IF;

(* 第五步：加减速 + 直线插补 → 写 Axis.Plan* *)
IF running AND (phase = 1 OR phase = 2 OR phase = 3) THEN

    sRem := 1.0 - s;

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

    (* 直线插补：Plan = Start + dir × (s × totalDist) *)
    Axis.PlanX := startX + dirX * s * totalDist;
    Axis.PlanY := startY + dirY * s * totalDist;
    Axis.PlanZ := startZ + dirZ * s * totalDist;

    (* 规划速度 = dir × sVel × totalDist *)
    Axis.PlanVX := dirX * sVel * totalDist;
    Axis.PlanVY := dirY * sVel * totalDist;
    Axis.PlanVZ := dirZ * sVel * totalDist;

    IF phase = 4 THEN
        Axis.PlanX := tgtX; Axis.PlanY := tgtY; Axis.PlanZ := tgtZ;
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
