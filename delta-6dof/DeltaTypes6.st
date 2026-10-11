(* ============================================================================
   文件名称 : DeltaTypes6.st
   功能描述 : Delta 6DOF 机器人运动控制数据类型定义（P2，对应 Python 黄金
              参考 test_delta6.py 的 DeltaAxis6 / kinematics6.py 的 PARAMS）
   ----------------------------------------------------------------------------
   一个 st_DeltaAxis6 实例代表一台六轴 Delta（三主臂+三副臂）：
     · 应用层  ：每周期写入反馈角 CurTheta[1..6]（rad，主1..3+副1..3）
     · 指令FB ：FB_PTP6/LIN6/CIR6 经 VAR_IN_OUT 共享，竞争 owner 令牌，
                写 PlanX..PlanVC
     · 核心FB ：FB_DeltaCore6 周期发布机械参数/使能，消费 Plan 做
                IK/FK-LM/前馈，输出 TarTheta[1..6] 与 CurX..CurC
   单位：角度对外用【度】（Plan/Cur 的 A/B/C、PlanVA/B/C），
         运动学 FB 边界转弧度；全部数值类型用 LREAL（双精度），
         以保证与 Python 双精度黄金参考逐拍一致。
   ============================================================================ *)

TYPE st_DeltaAxis6 :
STRUCT
    (* ===== 机械参数（核心FB每周期写入，指令FB只读） ===== *)
    R             : LREAL;    (* 主电机基座分布半径 (mm)                    *)
    r             : LREAL;    (* 平台主铰分布半径 (mm)                      *)
    L             : LREAL;    (* 主主动臂长度 (mm)                          *)
    l             : LREAL;    (* 主从动杆长度 (mm)                          *)
    Rac           : LREAL;    (* 副电机基座分布半径 (mm)                    *)
    ra            : LREAL;    (* 平台副铰分布半径 (mm)                      *)
    La            : LREAL;    (* 副主动臂长度 (mm)                          *)
    lb            : LREAL;    (* 副连杆长度 (mm)                            *)
    CycleTime     : LREAL;    (* 周期 (s)                                   *)
    FfEnable      : BOOL;     (* 速度前馈使能                               *)
    Kff           : LREAL;    (* 前馈增益 (0~1)                             *)
    Enable        : BOOL;     (* 核心使能（核心FB写，指令FB读）              *)

    (* ===== 反馈输入（应用层每周期写入，rad） ===== *)
    CurTheta      : ARRAY[1..6] OF LREAL;

    (* ===== 核心输出 ===== *)
    TarTheta      : ARRAY[1..6] OF LREAL;  (* 目标角（含前馈，rad）        *)
    CurX          : LREAL;    (* 当前位置 X（FK，mm）                        *)
    CurY          : LREAL;
    CurZ          : LREAL;
    CurA          : LREAL;    (* 当前姿态（FK，度）                          *)
    CurB          : LREAL;
    CurC          : LREAL;
    FkError       : BOOL;     (* FK 失败标志（保持上次位姿）                 *)

    (* ===== 调度状态（指令FB协调，核心只读） ===== *)
    Busy          : BOOL;     (* 轴组运动中（有活跃指令）                    *)
    ActiveType    : INT;      (* 0空闲 1=PTP 2=LIN 3=CIR                    *)
    owner         : DINT;     (* 当前控制权令牌                              *)
    cmdCount      : DINT;     (* 令牌计数器                                  *)
    InPos         : BOOL;     (* 轴级完成                                    *)
    hasBuf        : BOOL;     (* 已有一条缓存指令在等待                      *)

    (* ===== 规划输出（活跃指令FB写，核心FB消费） ===== *)
    PlanX         : LREAL;    (* mm *)
    PlanY         : LREAL;
    PlanZ         : LREAL;
    PlanA         : LREAL;    (* 度 *)
    PlanB         : LREAL;
    PlanC         : LREAL;
    PlanVX        : LREAL;    (* mm/s *)
    PlanVY        : LREAL;
    PlanVZ        : LREAL;
    PlanVA        : LREAL;    (* 度/s *)
    PlanVB        : LREAL;
    PlanVC        : LREAL;
END_STRUCT
END_TYPE
