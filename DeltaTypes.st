(* ============================================================================
   文件名称 : DeltaTypes.st
   功能描述 : Delta 机器人运动控制相关的数据类型定义
   ============================================================================ *)

(* ----------------------------------------------------------------------------
   共享轴状态结构（PLCopen 风格）
   ----------------------------------------------------------------------------
   一个 st_DeltaAxis 实例代表一台 Delta 机器人轴组，由三方共同访问：
     · 应用层    ：每周期写入反馈角度 CurTheta1..3
     · 指令 FB   ：FB_PTP / FB_LIN / FB_CIR 通过 VAR_IN_OUT 共享，
                   竞争轴控制权（owner 令牌 + BufferMode），并把各自的
                   规划结果写入 PlanX..PlanVZ
     · 核心 FB   ：FB_DeltaCore 周期写入机械参数与使能，消费 Plan 做
                   IK/FK/前馈，输出 TarTheta 与 CurX/Y/Z
   ---------------------------------------------------------------------------- *)
TYPE st_DeltaAxis :
STRUCT
    (* ===== 机械参数（核心FB每周期写入，指令FB只读，消除重复参数） ===== *)
    R             : REAL;     (* 静平台铰点分布圆半径 (mm)                    *)
    r             : REAL;     (* 动平台铰点分布圆半径 (mm)                    *)
    L             : REAL;     (* 主动臂长度 (mm)                              *)
    l             : REAL;     (* 从动臂长度 (mm)                              *)
    CycleTime     : REAL;     (* 周期 (s)                                     *)
    FfEnable      : BOOL;     (* 速度前馈使能                                 *)
    Kff           : REAL;     (* 前馈增益 (0~1)                               *)
    Enable        : BOOL;     (* 核心使能（核心FB写，指令FB读）                *)

    (* ===== 反馈输入（应用层每周期写入） ===== *)
    CurTheta1     : REAL;     (* 第 1 轴当前角度 (rad)                        *)
    CurTheta2     : REAL;
    CurTheta3     : REAL;

    (* ===== 核心输出 ===== *)
    TarTheta1     : REAL;     (* 第 1 轴目标角度 (rad，含前馈)                *)
    TarTheta2     : REAL;
    TarTheta3     : REAL;
    CurX          : REAL;     (* 当前位置 X（FK 实际位置，始终更新）          *)
    CurY          : REAL;
    CurZ          : REAL;

    (* ===== 调度状态（指令FB之间协调，核心只读） ===== *)
    Busy          : BOOL;     (* 轴组运动中（有活跃指令）                      *)
    ActiveType    : INT;      (* 活跃指令类型：0=空闲 1=PTP 2=LIN 3=CIR       *)
    owner         : DINT;     (* 当前控制权令牌（=持有者启动时的 cmdCount）    *)
    cmdCount      : DINT;     (* 令牌计数器，每次启动 +1                      *)
    InPos         : BOOL;     (* 轴级完成（活跃指令结束时置位）                *)
    hasBuf        : BOOL;     (* 已有一条缓存指令在等待（单缓存槽）            *)

    (* ===== 规划输出（活跃指令FB写，核心FB消费） ===== *)
    PlanX         : REAL;     (* 规划位置 X (mm)                              *)
    PlanY         : REAL;
    PlanZ         : REAL;
    PlanVX        : REAL;     (* 规划速度 X (mm/s)                            *)
    PlanVY        : REAL;
    PlanVZ        : REAL;
END_STRUCT
END_TYPE


(* ----------------------------------------------------------------------------
   运动状态枚举
   ---------------------------------------------------------------------------- *)
TYPE e_MoveState :
(
    MOVE_IDLE   := 0,     (* 空闲                                         *)
    MOVE_ACCEL  := 1,     (* 加速                                         *)
    MOVE_CONST  := 2,     (* 匀速                                         *)
    MOVE_DECEL  := 3,     (* 减速                                         *)
    MOVE_DONE   := 4      (* 完成                                         *)
);
END_TYPE
