(* ============================================================================
   功能块名称 : FB_DeltaCore  （核心层，唯一入口）
   功能描述 : Delta 机器人运动控制核心，一直周期调用
              1. 运动学（FK+IK+前馈）始终运行，维护当前位置和目标角度
              2. 应用层通过 cmd + Execute 下发运动指令
              3. 每条指令自带 BufferMode 指定处理方式：
                 0=拒绝：运动中来了新指令直接拒绝
                 1=打断：立即停止当前，从当前规划位置执行新指令
                 2=缓存：当前完成后自动执行缓存的指令（缓存槽满则拒绝）
              4. 所有输出由本FB统一维护
   ============================================================================ *)

FUNCTION_BLOCK FB_DeltaCore

VAR_INPUT
    R, r, L, l   : REAL;    (* 机械参数                              *)
    cmd          : st_MoveCmd;  (* 运动指令（含 BufferMode）            *)
    Execute      : BOOL;     (* 启动指令（上升沿）                    *)
    CurTheta1    : REAL;     (* 当前角度 1 (rad)                      *)
    CurTheta2    : REAL;
    CurTheta3    : REAL;
    CycleTime    : REAL;     (* 周期 (s)                              *)
    FfEnable     : BOOL;     (* 前馈使能                              *)
    Kff          : REAL;     (* 前馈增益                              *)
    Enable       : BOOL;     (* 使能                                  *)
END_VAR

VAR_OUTPUT
    TarTheta1    : REAL;     (* 目标角度 1 (rad)                      *)
    TarTheta2    : REAL;
    TarTheta3    : REAL;
    CurX         : REAL;     (* 当前位置 X (FK，始终更新)             *)
    CurY         : REAL;
    CurZ         : REAL;
    PlanX        : REAL;     (* 规划位置 X                            *)
    PlanY        : REAL;
    PlanZ        : REAL;
    InPos        : BOOL;     (* 指令完成                              *)
    Busy         : BOOL;     (* 运动中                                *)
    Rejected     : BOOL;     (* 指令被拒绝                            *)
    Buffered     : BOOL;     (* 有缓存指令等待                        *)
    ActiveType   : INT;      (* 当前运动类型 0=无 1=PTP 2=LIN 3=CIR  *)
END_VAR

VAR
    fbPTP : FB_PTP;
    fbLIN : FB_LIN;
    fbCIR : FB_CIR;
    fbKin : FB_DeltaKinematics;

    execPTP  : BOOL;
    execLIN  : BOOL;
    execCIR  : BOOL;
    exLast   : BOOL;
    exRise   : BOOL;
    planVX, planVY, planVZ : REAL;

    bufCmd   : st_MoveCmd;  (* 缓存指令                              *)
    hasBuf   : BOOL;        (* 有缓存指令                            *)
    bufActive: BOOL;        (* 正在执行缓存指令                      *)
    pendingStart: BOOL;     (* 待启动缓存指令（跨周期）              *)
    curType  : INT;         (* 当前运动类型(0/1/2)                   *)
    startType: INT;         (* 本周期要启动的运动类型(-1=不启动)     *)
    effCmd   : st_MoveCmd;  (* 当前生效的指令（cmd 或 bufCmd）       *)
END_VAR


(* ============================================================================
   失能处理
   ============================================================================ *)
IF NOT Enable THEN
    TarTheta1:=0.0; TarTheta2:=0.0; TarTheta3:=0.0;
    CurX:=0.0; CurY:=0.0; CurZ:=0.0;
    PlanX:=0.0; PlanY:=0.0; PlanZ:=0.0;
    InPos:=FALSE; Busy:=FALSE; Rejected:=FALSE; Buffered:=FALSE; ActiveType:=0;
    execPTP:=FALSE; execLIN:=FALSE; execCIR:=FALSE;
    hasBuf:=FALSE; bufActive:=FALSE; pendingStart:=FALSE; curType:=0; startType:=-1;
    RETURN;
END_IF;


(* ============================================================================
   Execute 上升沿检测
   ============================================================================ *)
exRise := Execute AND NOT exLast;
exLast := Execute;

Rejected := FALSE;
startType := -1;   (* 默认本周期不启动新运动 *)


(* ============================================================================
   指令处理（按 cmd.BufferMode）
   ----------------------------------------------------------------------------
   用 startType 统一协调：pendingStart（缓存自动启动）和 exRise（新指令）
   都通过设置 startType 来触发运动 FB 的 Execute。
   若两者同周期发生，exRise 的打断可覆盖 pendingStart。
   ============================================================================ *)

(* --- 第一步：处理待启动的缓存指令 --- *)
IF pendingStart THEN
    startType := bufCmd.MoveType;
    Busy := TRUE;
    InPos := FALSE;
    bufActive := TRUE;
    hasBuf := FALSE;
    pendingStart := FALSE;
END_IF;

(* --- 第二步：处理新指令 --- *)
IF exRise THEN
    IF (cmd.MoveType < 0) OR (cmd.MoveType > 2) THEN
        (* 非法运动类型：拒绝，避免启动后无对应FB执行导致Busy卡死 *)
        Rejected := TRUE;
    ELSIF Busy THEN
        CASE cmd.BufferMode OF
            0:  (* 拒绝 *)
                Rejected := TRUE;

            1:  (* 打断：覆盖 startType，清掉缓存 *)
                startType := cmd.MoveType;
                InPos := FALSE;
                hasBuf := FALSE;
                bufActive := FALSE;

            2:  (* 缓存 *)
                IF hasBuf THEN
                    Rejected := TRUE;   (* 缓存槽已满 *)
                ELSE
                    bufCmd := cmd;
                    hasBuf := TRUE;
                END_IF;
        END_CASE;
    ELSE
        (* 空闲：直接启动 *)
        startType := cmd.MoveType;
        Busy := TRUE;
        InPos := FALSE;
    END_IF;
END_IF;

(* --- 第三步：根据 startType 设置运动 FB 的 Execute 信号 --- *)
IF startType >= 0 THEN
    curType := startType;
    execPTP := (curType = 0);
    execLIN := (curType = 1);
    execCIR := (curType = 2);
    ActiveType := curType + 1;
ELSE
    execPTP := FALSE; execLIN := FALSE; execCIR := FALSE;
END_IF;

Buffered := hasBuf;

(* 选择当前生效的指令：正在执行缓存指令时用 bufCmd，否则用 cmd *)
IF bufActive THEN
    effCmd := bufCmd;
ELSE
    effCmd := cmd;
END_IF;


(* ============================================================================
   调用运动规划FB
   ============================================================================ *)
fbPTP(
    R:=R, r:=r, L:=L, l:=l,
    TargetX:=effCmd.PosX, TargetY:=effCmd.PosY, TargetZ:=effCmd.PosZ,
    CurTheta1:=CurTheta1, CurTheta2:=CurTheta2, CurTheta3:=CurTheta3,
    Speed:=effCmd.Speed, Accel:=effCmd.Accel, Jerk:=effCmd.Jerk,
    MoveMode:=effCmd.MoveMode, CycleTime:=CycleTime,
    Enable:=Enable, Execute:=execPTP
);

fbLIN(
    TargetX:=effCmd.PosX, TargetY:=effCmd.PosY, TargetZ:=effCmd.PosZ,
    Speed:=effCmd.Speed, Accel:=effCmd.Accel, Jerk:=effCmd.Jerk,
    MoveMode:=effCmd.MoveMode, CycleTime:=CycleTime,
    Enable:=Enable, Execute:=execLIN,
    InitX:=CurX, InitY:=CurY, InitZ:=CurZ
);

fbCIR(
    TargetX:=effCmd.PosX, TargetY:=effCmd.PosY, TargetZ:=effCmd.PosZ,
    AuxX:=effCmd.AuxX, AuxY:=effCmd.AuxY, AuxZ:=effCmd.AuxZ,
    CirMode:=effCmd.CirMode,
    Speed:=effCmd.Speed, Accel:=effCmd.Accel, Jerk:=effCmd.Jerk,
    MoveMode:=effCmd.MoveMode, CycleTime:=CycleTime,
    Enable:=Enable, Execute:=execCIR,
    InitX:=CurX, InitY:=CurY, InitZ:=CurZ
);


(* ============================================================================
   选择当前运动FB的规划位置和速度
   ============================================================================ *)
CASE curType OF
    0:
        PlanX := fbPTP.PlanX; PlanY := fbPTP.PlanY; PlanZ := fbPTP.PlanZ;
        planVX := fbPTP.PlanVX; planVY := fbPTP.PlanVY; planVZ := fbPTP.PlanVZ;
    1:
        PlanX := fbLIN.PlanX; PlanY := fbLIN.PlanY; PlanZ := fbLIN.PlanZ;
        planVX := fbLIN.PlanVX; planVY := fbLIN.PlanVY; planVZ := fbLIN.PlanVZ;
    2:
        PlanX := fbCIR.PlanX; PlanY := fbCIR.PlanY; PlanZ := fbCIR.PlanZ;
        planVX := fbCIR.PlanVX; planVY := fbCIR.PlanVY; planVZ := fbCIR.PlanVZ;
END_CASE;


(* ============================================================================
   调用基础运动学（始终运行）
   ============================================================================ *)
fbKin(
    R:=R, r:=r, L:=L, l:=l,
    PlanX:=PlanX, PlanY:=PlanY, PlanZ:=PlanZ,
    PlanVX:=planVX, PlanVY:=planVY, PlanVZ:=planVZ,
    FfEnable:=FfEnable, Kff:=Kff,
    CurTheta1:=CurTheta1, CurTheta2:=CurTheta2, CurTheta3:=CurTheta3,
    CycleTime:=CycleTime, Enable:=Enable
);

TarTheta1 := fbKin.TarTheta1;
TarTheta2 := fbKin.TarTheta2;
TarTheta3 := fbKin.TarTheta3;
CurX := fbKin.CurX;
CurY := fbKin.CurY;
CurZ := fbKin.CurZ;


(* ============================================================================
   运动完成检测 + 缓存指令待启动
   ============================================================================ *)
IF Busy THEN
    CASE curType OF
        0:  IF fbPTP.InPos THEN InPos := TRUE; END_IF;
        1:  IF fbLIN.InPos THEN InPos := TRUE; END_IF;
        2:  IF fbCIR.InPos THEN InPos := TRUE; END_IF;
    END_CASE;

    IF InPos THEN
        Busy := FALSE;
        ActiveType := 0;
        bufActive := FALSE;

        (* 有缓存指令则标记待启动，下一周期生效 *)
        IF hasBuf THEN
            pendingStart := TRUE;
        END_IF;
    END_IF;
END_IF;

END_FUNCTION_BLOCK
