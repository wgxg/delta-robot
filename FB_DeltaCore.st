(* ============================================================================
   功能块名称 : FB_DeltaCore  （纯运动学引擎，一直周期调用）
   功能描述 : Delta 机器人核心层，只做运动学，不做指令调度
              1. 每周期把机械参数/使能写入共享 Axis，供指令FB读取（参数唯一来源）
              2. 消费活跃指令FB写入的 Axis.Plan*，调用 FB_DeltaKinematics
                 完成 IK/FK/速度前馈，输出目标角度 TarTheta
              3. FK 始终运行，维护当前实际位置 CurX/Y/Z
              4. 空闲（Axis.Busy=FALSE，无指令执行）时：
                   Plan = CurX/Y/Z（跟随FK实际位置），PlanV = 0
                   TarTheta = CurTheta（1:1 跟随当前输入角度）
                 有指令执行时：
                   TarTheta = IK(Plan) + 前馈
   调用约定 : 每个扫描周期先调用所有指令FB（FB_PTP/FB_LIN/FB_CIR），
              最后调用本FB。调度（Execute/BufferMode/Busy/InPos/打断/缓存）
              全部由指令FB通过共享 Axis 自行协调，本FB不参与。
   ============================================================================ *)

FUNCTION_BLOCK FB_DeltaCore

VAR_INPUT
    R, r, L, l   : REAL;    (* 机械参数（写入Axis，全系统唯一来源）    *)
    CycleTime    : REAL;    (* 周期 (s)                              *)
    FfEnable     : BOOL;    (* 前馈使能                              *)
    Kff          : REAL;    (* 前馈增益                              *)
    Enable       : BOOL;    (* 使能                                  *)
END_VAR

VAR_IN_OUT
    Axis         : st_DeltaAxis;   (* 共享轴状态（与指令FB同一个实例） *)
END_VAR

VAR
    fbKin : FB_DeltaKinematics;
END_VAR


(* ============================================================================
   失能：清零输出与调度状态，通知指令FB复位
   ============================================================================ *)
IF NOT Enable THEN
    Axis.Enable := FALSE;
    Axis.R:=R; Axis.r:=r; Axis.L:=L; Axis.l:=l;
    Axis.TarTheta1:=0.0; Axis.TarTheta2:=0.0; Axis.TarTheta3:=0.0;
    Axis.CurX:=0.0; Axis.CurY:=0.0; Axis.CurZ:=0.0;
    Axis.PlanX:=0.0; Axis.PlanY:=0.0; Axis.PlanZ:=0.0;
    Axis.PlanVX:=0.0; Axis.PlanVY:=0.0; Axis.PlanVZ:=0.0;
    Axis.Busy:=FALSE; Axis.ActiveType:=0; Axis.owner:=0;
    Axis.cmdCount:=0; Axis.hasBuf:=FALSE; Axis.InPos:=FALSE;
    RETURN;
END_IF;


(* ============================================================================
   1. 发布机械参数/使能到共享轴（指令FB下周期即可见）
   ============================================================================ *)
Axis.R:=R; Axis.r:=r; Axis.L:=L; Axis.l:=l;
Axis.CycleTime:=CycleTime;
Axis.FfEnable:=FfEnable; Axis.Kff:=Kff;
Axis.Enable:=TRUE;


(* ============================================================================
   2. 空闲跟随：无活跃指令时规划位置=FK实际位置，规划速度=0
      （有活跃指令时 Plan* 已由该指令FB在本周期写好，保持不动）
   ============================================================================ *)
IF NOT Axis.Busy THEN
    Axis.PlanX := Axis.CurX;
    Axis.PlanY := Axis.CurY;
    Axis.PlanZ := Axis.CurZ;
    Axis.PlanVX := 0.0;
    Axis.PlanVY := 0.0;
    Axis.PlanVZ := 0.0;
END_IF;


(* ============================================================================
   3. 基础运动学（始终运行）：IK(Plan)+前馈 / FK(CurTheta)
   ============================================================================ *)
fbKin(
    R:=R, r:=r, L:=L, l:=l,
    PlanX:=Axis.PlanX, PlanY:=Axis.PlanY, PlanZ:=Axis.PlanZ,
    PlanVX:=Axis.PlanVX, PlanVY:=Axis.PlanVY, PlanVZ:=Axis.PlanVZ,
    FfEnable:=FfEnable, Kff:=Kff,
    CurTheta1:=Axis.CurTheta1, CurTheta2:=Axis.CurTheta2, CurTheta3:=Axis.CurTheta3,
    CycleTime:=CycleTime, Enable:=Enable
);

(* FK 实际位置始终更新 *)
Axis.CurX := fbKin.CurX;
Axis.CurY := fbKin.CurY;
Axis.CurZ := fbKin.CurZ;

(* 执行指令：目标角度=IK(Plan)+前馈；空闲：目标角度=当前角度（1:1跟随） *)
IF Axis.Busy THEN
    Axis.TarTheta1 := fbKin.TarTheta1;
    Axis.TarTheta2 := fbKin.TarTheta2;
    Axis.TarTheta3 := fbKin.TarTheta3;
ELSE
    Axis.TarTheta1 := Axis.CurTheta1;
    Axis.TarTheta2 := Axis.CurTheta2;
    Axis.TarTheta3 := Axis.CurTheta3;
END_IF;

END_FUNCTION_BLOCK
