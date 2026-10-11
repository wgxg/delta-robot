(* ============================================================================
   功能块名称 : FB_DeltaCore6（纯运动学引擎，一直周期调用）
   功能描述 : 对应 Python 黄金参考 test_delta6.py 的
              FB_DeltaCore6 + FB_DeltaKinematics6.__call__：
                1. 发布机械参数/使能到共享 Axis（参数唯一来源）
                2. FK：冷启动纯牛顿(DEFAULT_GUESS)；之后 LM 跟踪且【Busy 时
                   以 Plan 为初值锚定装配页】（见 DESIGN 3.3），失败兜底纯牛顿
                3. IK：Plan→6 目标角（闭式；失败整体保持上拍 tar）
                4. 前馈：解析 ik_vel；Tar = tar + Kff·q̇·dt
                5. 空闲：Plan=Cur（度）、PlanV=0、Tar=CurTheta 1:1 跟随
                6. 失能：全清零并复位内部 FK 状态（冷启动标志置位）
   调用约定 : 每周期先调指令 FB，最后调本 FB。本 FB 不参与任何调度。
   ============================================================================ *)

FUNCTION_BLOCK FB_DeltaCore6
VAR_INPUT
    R,r,L,l       : LREAL;
    Rac,ra,La,lb  : LREAL;
    CycleTime     : LREAL;
    FfEnable      : BOOL;
    Kff           : LREAL;
    Enable        : BOOL;
END_VAR
VAR_IN_OUT
    Axis : st_DeltaAxis6;
END_VAR
VAR
    fbKin : FB_DeltaKinematics6;

    (* FK/IK 持久状态（对应 Python FB_DeltaKinematics6 成员） *)
    tar   : ARRAY[1..6] OF LREAL := (1.13,1.13,1.13,0.78,0.78,0.78);
    guess : ARRAY[1..6] OF LREAL := (0.0,0.0,300.0,0.09,0.06,0.04);
    cur   : ARRAY[1..6] OF LREAL := (0.0,0.0,300.0,0.09,0.06,0.04);
    cold  : BOOL := TRUE;

    curAng : ARRAY[1..6] OF LREAL;
    pose   : ARRAY[1..6] OF LREAL;   (* Plan 的弧度表示 *)
    twist  : ARRAY[1..6] OF LREAL;
    dth    : ARRAY[1..6] OF LREAL;
    k      : INT;
    IK_OK, FK_OK : BOOL;
END_VAR


(* ============================================================================
   失能：清零输出与调度状态，复位 FK 持久状态
   ============================================================================ *)
IF NOT Enable THEN
    Axis.Enable:=FALSE;
    Axis.R:=R; Axis.r:=r; Axis.L:=L; Axis.l:=l;
    Axis.Rac:=Rac; Axis.ra:=ra; Axis.La:=La; Axis.lb:=lb;
    FOR k:=1 TO 6 DO Axis.TarTheta[k]:=0.0; END_FOR;
    Axis.CurX:=0.0; Axis.CurY:=0.0; Axis.CurZ:=0.0;
    Axis.CurA:=0.0; Axis.CurB:=0.0; Axis.CurC:=0.0;
    Axis.FkError:=FALSE;
    Axis.PlanX:=0.0; Axis.PlanY:=0.0; Axis.PlanZ:=0.0;
    Axis.PlanA:=0.0; Axis.PlanB:=0.0; Axis.PlanC:=0.0;
    Axis.PlanVX:=0.0; Axis.PlanVY:=0.0; Axis.PlanVZ:=0.0;
    Axis.PlanVA:=0.0; Axis.PlanVB:=0.0; Axis.PlanVC:=0.0;
    Axis.Busy:=FALSE; Axis.ActiveType:=0; Axis.owner:=0;
    Axis.cmdCount:=0; Axis.hasBuf:=FALSE; Axis.InPos:=FALSE;

    tar[1]:=1.13; tar[2]:=1.13; tar[3]:=1.13;
    tar[4]:=0.78; tar[5]:=0.78; tar[6]:=0.78;
    cur[1]:=0.0; cur[2]:=0.0; cur[3]:=300.0;
    cur[4]:=0.09; cur[5]:=0.06; cur[6]:=0.04;
    guess[1]:=0.0; guess[2]:=0.0; guess[3]:=300.0;
    guess[4]:=0.09; guess[5]:=0.06; guess[6]:=0.04;
    cold:=TRUE;
    RETURN;
END_IF;


(* ============================================================================
   1. 发布机械参数/使能
   ============================================================================ *)
Axis.R:=R; Axis.r:=r; Axis.L:=L; Axis.l:=l;
Axis.Rac:=Rac; Axis.ra:=ra; Axis.La:=La; Axis.lb:=lb;
Axis.CycleTime:=CycleTime;
Axis.FfEnable:=FfEnable; Axis.Kff:=Kff;
Axis.Enable:=TRUE;

FOR k:=1 TO 6 DO curAng[k]:=Axis.CurTheta[k]; END_FOR;


(* ============================================================================
   2. FK：6 反馈角 → 当前位姿
   ============================================================================ *)
IF cold THEN
    fbKin(Mode:=2,
          R:=R, r:=r, L:=L, l:=l, Rac:=Rac, ra:=ra, La:=La, lb:=lb,
          Ang6:=curAng, Guess6:=guess);
ELSE
    (* Busy 时 Plan 是活跃指令的规划位姿；空闲时 Plan 是上拍 Cur（本拍空闲
       覆盖发生在 FK 之后）——两种情形都用 Plan 锚定装配页 *)
    pose[1]:=Axis.PlanX; pose[2]:=Axis.PlanY; pose[3]:=Axis.PlanZ;
    pose[4]:=RAD(Axis.PlanA); pose[5]:=RAD(Axis.PlanB); pose[6]:=RAD(Axis.PlanC);
    fbKin(Mode:=1,
          R:=R, r:=r, L:=L, l:=l, Rac:=Rac, ra:=ra, La:=La, lb:=lb,
          Ang6:=curAng, Guess6:=pose);
    IF NOT fbKin.OK THEN
        fbKin(Mode:=2,
              R:=R, r:=r, L:=L, l:=l, Rac:=Rac, ra:=ra, La:=La, lb:=lb,
              Ang6:=curAng, Guess6:=pose);
    END_IF;
END_IF;

IF fbKin.OK THEN
    FK_OK:=TRUE;
    Axis.FkError:=FALSE;
    FOR k:=1 TO 6 DO
        cur[k]:=fbKin.OutVec6[k];
        guess[k]:=fbKin.OutVec6[k];
    END_FOR;
    cold:=FALSE;
ELSE
    FK_OK:=FALSE;
    Axis.FkError:=TRUE;
END_IF;


(* ============================================================================
   3. IK：规划位姿 → 6 目标角（失败整体保持 tar）
   ============================================================================ *)
pose[1]:=Axis.PlanX; pose[2]:=Axis.PlanY; pose[3]:=Axis.PlanZ;
pose[4]:=RAD(Axis.PlanA); pose[5]:=RAD(Axis.PlanB); pose[6]:=RAD(Axis.PlanC);
fbKin(Mode:=3,
      R:=R, r:=r, L:=L, l:=l, Rac:=Rac, ra:=ra, La:=La, lb:=lb,
      Pose6:=pose, Ref6:=tar);
IF fbKin.OK THEN
    IK_OK:=TRUE;
    FOR k:=1 TO 6 DO tar[k]:=fbKin.OutVec6[k]; END_FOR;
ELSE
    IK_OK:=FALSE;
END_IF;


(* ============================================================================
   4. 前馈：q̇ = ik_vel(plan, tar, twist)；dth = q̇·CycleTime
   ============================================================================ *)
FOR k:=1 TO 6 DO dth[k]:=0.0; END_FOR;
IF Axis.FfEnable AND IK_OK THEN
    twist[1]:=Axis.PlanVX; twist[2]:=Axis.PlanVY; twist[3]:=Axis.PlanVZ;
    twist[4]:=RAD(Axis.PlanVA); twist[5]:=RAD(Axis.PlanVB); twist[6]:=RAD(Axis.PlanVC);
    fbKin(Mode:=4,
          R:=R, r:=r, L:=L, l:=l, Rac:=Rac, ra:=ra, La:=La, lb:=lb,
          Pose6:=pose, Ang6:=tar, Twist6:=twist);
    FOR k:=1 TO 6 DO dth[k]:=fbKin.OutVec6[k]*Axis.CycleTime; END_FOR;
END_IF;

FOR k:=1 TO 6 DO
    Axis.TarTheta[k]:=tar[k]+Axis.Kff*dth[k];
END_FOR;


(* ============================================================================
   5. FK 结果回写轴（姿态转度）
   ============================================================================ *)
Axis.CurX:=cur[1]; Axis.CurY:=cur[2]; Axis.CurZ:=cur[3];
Axis.CurA:=DEG(cur[4]); Axis.CurB:=DEG(cur[5]); Axis.CurC:=DEG(cur[6]);


(* ============================================================================
   6. 空闲跟随：Plan=Cur、PlanV=0、Tar=CurTheta（1:1）
   ============================================================================ *)
IF NOT Axis.Busy THEN
    Axis.PlanX:=Axis.CurX; Axis.PlanY:=Axis.CurY; Axis.PlanZ:=Axis.CurZ;
    Axis.PlanA:=Axis.CurA; Axis.PlanB:=Axis.CurB; Axis.PlanC:=Axis.CurC;
    Axis.PlanVX:=0.0; Axis.PlanVY:=0.0; Axis.PlanVZ:=0.0;
    Axis.PlanVA:=0.0; Axis.PlanVB:=0.0; Axis.PlanVC:=0.0;
    FOR k:=1 TO 6 DO Axis.TarTheta[k]:=Axis.CurTheta[k]; END_FOR;
END_IF;

END_FUNCTION_BLOCK
