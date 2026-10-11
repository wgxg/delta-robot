(* ============================================================================
   功能块名称 : FB_PTP6（点到点，关节空间 6 轴归一化 s 同步插补）
   功能描述 : 对应 Python 黄金参考 test_delta6.py 的 FB_PTP6。
              · 上升沿 IK 预检，非法（不可达/参数<=0）当周期 Rejected
              · 启动：6 关节角 startTh→tgtTh 线性 s 插值，每拍经 fk_track
                （Mode=1，guess=fkg 沿程传递）求规划位姿；中心差分求速度
              · 缓存提升在后续周期发生：起终点关节角按提升时刻 Cur 重算
              · 调度语义（owner/BufferMode 0/1/2/信号锁存）与 3DOF 逐行一致
   ============================================================================ *)

FUNCTION_BLOCK FB_PTP6
VAR_INPUT
    Execute      : BOOL;
    PositionX, PositionY, PositionZ : LREAL;
    PositionA, PositionB, PositionC : LREAL;   (* 度 *)
    Speed, Accel, Jerk : LREAL;
    MoveMode     : INT;        (* 0=梯形 1=S曲线 *)
    BufferMode   : INT;        (* 0=拒绝 1=打断 2=缓存 *)
END_VAR
VAR_IN_OUT
    Axis : st_DeltaAxis6;
END_VAR
VAR_OUTPUT
    Busy, InPos, CommandAborted, Rejected : BOOL;
END_VAR
VAR
    kin : FB_DeltaKinematics6;

    (* 几何/轨迹状态 *)
    tgtDeg  : ARRAY[1..6] OF LREAL := (0.0,0.0,300.0,0.0,0.0,0.0);
    curAng  : ARRAY[1..6] OF LREAL;
    startTh : ARRAY[1..6] OF LREAL;
    tgtTh   : ARRAY[1..6] OF LREAL;
    ang     : ARRAY[1..6] OF LREAL;
    fkg     : ARRAY[1..6] OF LREAL := (0.0,0.0,300.0,0.0,0.0,0.0);
    poseR   : ARRAY[1..6] OF LREAL;
    curR    : ARRAY[1..6] OF LREAL;
    pr,p0v,p1v,rates : ARRAY[1..6] OF LREAL;

    s, sVel, sAcc : LREAL;
    maxSVel, maxSAcc, maxSJerk : LREAL;
    sStop, sRem : LREAL;
    total, dpos, drot : LREAL;
    ds, sp0, sp1 : LREAL;
    phase : INT;
    poseOK, geomOK, tgtFail : BOOL;
    k : INT;

    (* RPY→四元数/短弧夹角 内联临时量（w,x,y,z） *)
    ca,sa,cb,sb,cc,sc : LREAL;
    qw,qx,qy,qz : LREAL;          (* tgt 四元数 *)
    cw,cx,cy,cz : LREAL;          (* cur 四元数 *)
    dotq : LREAL;

    (* 调度状态 *)
    myId : DINT;
    running, waiting, wantStart, wasWaiting : BOOL;
    exLast, exRise : BOOL;
END_VAR


(* ============================================================================
   失能：复位自身，不写规划
   ============================================================================ *)
IF NOT Axis.Enable THEN
    Busy:=FALSE; InPos:=FALSE; CommandAborted:=FALSE; Rejected:=FALSE;
    running:=FALSE; waiting:=FALSE; phase:=0; s:=0.0;
    exLast:=Execute;
    RETURN;
END_IF;

Rejected:=FALSE;
exRise:=Execute AND NOT exLast;
exLast:=Execute;
wantStart:=FALSE;

(* ============================================================================
   第一步：上升沿仲裁（含 IK 预检）
   ============================================================================ *)
IF exRise THEN
    wasWaiting:=waiting;
    IF waiting THEN
        Axis.hasBuf:=FALSE;
        waiting:=FALSE; Busy:=FALSE;
    END_IF;

    InPos:=FALSE; CommandAborted:=FALSE;

    tgtDeg[1]:=PositionX; tgtDeg[2]:=PositionY; tgtDeg[3]:=PositionZ;
    tgtDeg[4]:=PositionA; tgtDeg[5]:=PositionB; tgtDeg[6]:=PositionC;
    FOR k:=1 TO 6 DO curAng[k]:=Axis.CurTheta[k]; END_FOR;
    poseR[1]:=PositionX; poseR[2]:=PositionY; poseR[3]:=PositionZ;
    poseR[4]:=RAD(PositionA); poseR[5]:=RAD(PositionB); poseR[6]:=RAD(PositionC);
    kin(Mode:=3,
        R:=Axis.R, r:=Axis.r, L:=Axis.L, l:=Axis.l,
        Rac:=Axis.Rac, ra:=Axis.ra, La:=Axis.La, lb:=Axis.lb,
        Pose6:=poseR, Ref6:=curAng);
    geomOK:=kin.OK;
    IF geomOK THEN
        FOR k:=1 TO 6 DO
            tgtTh[k]:=kin.OutVec6[k];
            startTh[k]:=curAng[k];
        END_FOR;
    END_IF;

    IF (Speed<=0.0) OR (Accel<=0.0) OR ((MoveMode=1) AND (Jerk<=0.0))
       OR (NOT geomOK) THEN
        Rejected:=TRUE;
        IF wasWaiting THEN CommandAborted:=TRUE; END_IF;
    ELSIF running AND (Axis.owner=myId) THEN
        wantStart:=TRUE;
    ELSIF Axis.Busy THEN
        IF BufferMode=0 THEN
            Rejected:=TRUE;
            IF wasWaiting THEN CommandAborted:=TRUE; END_IF;
        ELSIF BufferMode=1 THEN
            wantStart:=TRUE;
        ELSIF BufferMode=2 THEN
            IF Axis.hasBuf THEN
                Rejected:=TRUE;
                IF wasWaiting THEN CommandAborted:=TRUE; END_IF;
            ELSE
                waiting:=TRUE; Axis.hasBuf:=TRUE; Busy:=TRUE;
            END_IF;
        END_IF;
    ELSE
        wantStart:=TRUE;
    END_IF;
END_IF;

(* ============================================================================
   第二步：缓存等待处理
   ============================================================================ *)
IF waiting THEN
    IF NOT Axis.hasBuf THEN
        waiting:=FALSE; Busy:=FALSE; CommandAborted:=TRUE;
    ELSIF NOT Axis.Busy THEN
        waiting:=FALSE; Axis.hasBuf:=FALSE; wantStart:=TRUE;
    END_IF;
END_IF;

(* ============================================================================
   第三步：统一启动（令牌 + 轨迹初始化；提升时刻重算关节角）
   ============================================================================ *)
IF wantStart THEN
    Axis.cmdCount:=Axis.cmdCount+1;
    myId:=Axis.cmdCount;
    Axis.owner:=myId;
    Axis.ActiveType:=1;
    Axis.Busy:=TRUE;
    Axis.InPos:=FALSE;
    Axis.hasBuf:=FALSE;
    running:=TRUE; Busy:=TRUE; InPos:=FALSE;

    FOR k:=1 TO 6 DO curAng[k]:=Axis.CurTheta[k]; END_FOR;
    kin(Mode:=3,
        R:=Axis.R, r:=Axis.r, L:=Axis.L, l:=Axis.l,
        Rac:=Axis.Rac, ra:=Axis.ra, La:=Axis.La, lb:=Axis.lb,
        Pose6:=poseR, Ref6:=curAng);
    tgtFail:=NOT kin.OK;
    IF kin.OK THEN
        FOR k:=1 TO 6 DO
            startTh[k]:=curAng[k];
            tgtTh[k]:=kin.OutVec6[k];
        END_FOR;
    END_IF;

    (* total = hypot(平移mm, 100*短弧转角rad)，姿态用四元数短弧 *)
    curR[1]:=Axis.CurX; curR[2]:=Axis.CurY; curR[3]:=Axis.CurZ;
    curR[4]:=RAD(Axis.CurA); curR[5]:=RAD(Axis.CurB); curR[6]:=RAD(Axis.CurC);
    dpos:=0.0;
    FOR k:=1 TO 3 DO dpos:=dpos+(poseR[k]-curR[k])*(poseR[k]-curR[k]); END_FOR;
    dpos:=SQRT(dpos);

    ca:=COS(curR[4]*0.5); sa:=SIN(curR[4]*0.5);
    cb:=COS(curR[5]*0.5); sb:=SIN(curR[5]*0.5);
    cc:=COS(curR[6]*0.5); sc:=SIN(curR[6]*0.5);
    cw:=cc*cb*ca+sc*sb*sa;
    cx:=cc*cb*sa-sc*sb*ca;
    cy:=cc*sb*ca+sc*cb*sa;
    cz:=sc*cb*ca-cc*sb*sa;

    ca:=COS(poseR[4]*0.5); sa:=SIN(poseR[4]*0.5);
    cb:=COS(poseR[5]*0.5); sb:=SIN(poseR[5]*0.5);
    cc:=COS(poseR[6]*0.5); sc:=SIN(poseR[6]*0.5);
    qw:=cc*cb*ca+sc*sb*sa;
    qx:=cc*cb*sa-sc*sb*ca;
    qy:=cc*sb*ca+sc*cb*sa;
    qz:=sc*cb*ca-cc*sb*sa;

    dotq:=cw*qw+cx*qx+cy*qy+cz*qz;
    IF dotq<0.0 THEN dotq:=-dotq; END_IF;
    IF dotq>1.0 THEN dotq:=1.0; END_IF;
    drot:=2.0*ACOS(dotq);

    total:=SQRT(dpos*dpos+(100.0*drot)*(100.0*drot));
    FOR k:=1 TO 6 DO fkg[k]:=curR[k]; END_FOR;

    IF tgtFail OR (total<1.0E-4) THEN
        Axis.PlanX:=tgtDeg[1]; Axis.PlanY:=tgtDeg[2]; Axis.PlanZ:=tgtDeg[3];
        Axis.PlanA:=tgtDeg[4]; Axis.PlanB:=tgtDeg[5]; Axis.PlanC:=tgtDeg[6];
        Axis.PlanVX:=0.0; Axis.PlanVY:=0.0; Axis.PlanVZ:=0.0;
        Axis.PlanVA:=0.0; Axis.PlanVB:=0.0; Axis.PlanVC:=0.0;
        s:=1.0; phase:=4;
    ELSE
        maxSVel:=Speed/total;
        maxSAcc:=Accel/total;
        maxSJerk:=Jerk/total;
        s:=0.0; sVel:=0.0; sAcc:=0.0; phase:=1;
    END_IF;
END_IF;

(* ============================================================================
   第四步：被打断（令牌易主）
   ============================================================================ *)
IF running AND (Axis.owner<>myId) THEN
    running:=FALSE; phase:=0; Busy:=FALSE; CommandAborted:=TRUE;
END_IF;

(* ============================================================================
   第五步：轨迹推进 + 关节插值 + FK 求规划位姿
   ============================================================================ *)
IF running AND (phase=1 OR phase=2 OR phase=3) THEN

    (* 梯形/S 曲线 profile（对应 _profile） *)
    sRem:=1.0-s;
    IF MoveMode=0 THEN
        sStop:=sVel*sVel/(2.0*maxSAcc);
    ELSE
        sStop:=sVel*sVel/(2.0*maxSAcc)+sVel*maxSAcc/(2.0*maxSJerk);
    END_IF;

    IF phase=1 THEN
        IF sRem<=sStop THEN
            phase:=3;
        ELSE
            IF MoveMode=0 THEN
                sAcc:=maxSAcc;
            ELSE
                sAcc:=sAcc+maxSJerk*Axis.CycleTime;
                IF sAcc>maxSAcc THEN sAcc:=maxSAcc; END_IF;
            END_IF;
            sVel:=sVel+sAcc*Axis.CycleTime;
            IF sVel>=maxSVel THEN
                sVel:=maxSVel; sAcc:=0.0; phase:=2;
            END_IF;
        END_IF;
    ELSIF phase=2 THEN
        IF sRem<=sStop THEN
            phase:=3;
        ELSE
            sVel:=maxSVel; sAcc:=0.0;
        END_IF;
    ELSIF phase=3 THEN
        IF MoveMode=0 THEN
            sAcc:=-maxSAcc;
        ELSE
            sAcc:=sAcc-maxSJerk*Axis.CycleTime;
            IF sAcc<-maxSAcc THEN sAcc:=-maxSAcc; END_IF;
        END_IF;
        sVel:=sVel+sAcc*Axis.CycleTime;
        IF sVel<=0.0 THEN
            sVel:=0.0; sAcc:=0.0; s:=1.0; phase:=4;
        END_IF;
    END_IF;

    IF phase<>4 THEN
        s:=s+sVel*Axis.CycleTime;
        IF s>1.0 THEN s:=1.0; END_IF;
    END_IF;

    IF phase<>4 THEN
        (* --- 主 pose：_plan_pose(s)，fk_track 失败兜底纯牛顿 --- *)
        poseOK:=FALSE;
        FOR k:=1 TO 6 DO ang[k]:=startTh[k]+(tgtTh[k]-startTh[k])*s; END_FOR;
        kin(Mode:=1,
            R:=Axis.R, r:=Axis.r, L:=Axis.L, l:=Axis.l,
            Rac:=Axis.Rac, ra:=Axis.ra, La:=Axis.La, lb:=Axis.lb,
            Ang6:=ang, Guess6:=fkg);
        IF kin.OK THEN
            poseOK:=TRUE;
        ELSE
            kin(Mode:=2,
                R:=Axis.R, r:=Axis.r, L:=Axis.L, l:=Axis.l,
                Rac:=Axis.Rac, ra:=Axis.ra, La:=Axis.La, lb:=Axis.lb,
                Ang6:=ang, Guess6:=fkg);
            poseOK:=kin.OK;
        END_IF;
        IF poseOK THEN
            FOR k:=1 TO 6 DO
                fkg[k]:=kin.OutVec6[k];
                pr[k]:=kin.OutVec6[k];
            END_FOR;

            (* --- fd_rates：s0=s 处 _plan_pose（再次推进 fkg） --- *)
            sp0:=s;
            IF sp0>1.0 THEN sp0:=1.0; END_IF;
            FOR k:=1 TO 6 DO ang[k]:=startTh[k]+(tgtTh[k]-startTh[k])*sp0; END_FOR;
            kin(Mode:=1,
                R:=Axis.R, r:=Axis.r, L:=Axis.L, l:=Axis.l,
                Rac:=Axis.Rac, ra:=Axis.ra, La:=Axis.La, lb:=Axis.lb,
                Ang6:=ang, Guess6:=fkg);
            IF NOT kin.OK THEN
                kin(Mode:=2,
                    R:=Axis.R, r:=Axis.r, L:=Axis.L, l:=Axis.l,
                    Rac:=Axis.Rac, ra:=Axis.ra, La:=Axis.La, lb:=Axis.lb,
                    Ang6:=ang, Guess6:=fkg);
            END_IF;
            IF kin.OK THEN
                FOR k:=1 TO 6 DO
                    fkg[k]:=kin.OutVec6[k];
                    p0v[k]:=kin.OutVec6[k];
                END_FOR;
            END_IF;

            (* --- s1=min(s+1e-6,1) 处 _plan_pose --- *)
            sp1:=s+1.0E-6;
            IF sp1>1.0 THEN sp1:=1.0; END_IF;
            IF sp1<=sp0 THEN
                FOR k:=1 TO 6 DO rates[k]:=0.0; END_FOR;
            ELSE
                FOR k:=1 TO 6 DO ang[k]:=startTh[k]+(tgtTh[k]-startTh[k])*sp1; END_FOR;
                kin(Mode:=1,
                    R:=Axis.R, r:=Axis.r, L:=Axis.L, l:=Axis.l,
                    Rac:=Axis.Rac, ra:=Axis.ra, La:=Axis.La, lb:=Axis.lb,
                    Ang6:=ang, Guess6:=fkg);
                IF NOT kin.OK THEN
                    kin(Mode:=2,
                        R:=Axis.R, r:=Axis.r, L:=Axis.L, l:=Axis.l,
                        Rac:=Axis.Rac, ra:=Axis.ra, La:=Axis.La, lb:=Axis.lb,
                        Ang6:=ang, Guess6:=fkg);
                END_IF;
                IF kin.OK THEN
                    FOR k:=1 TO 6 DO
                        fkg[k]:=kin.OutVec6[k];
                        p1v[k]:=kin.OutVec6[k];
                        rates[k]:=(p1v[k]-p0v[k])/1.0E-6*sVel;
                    END_FOR;
                END_IF;
            END_IF;

            Axis.PlanX:=pr[1]; Axis.PlanY:=pr[2]; Axis.PlanZ:=pr[3];
            Axis.PlanA:=DEG(pr[4]); Axis.PlanB:=DEG(pr[5]); Axis.PlanC:=DEG(pr[6]);
            Axis.PlanVX:=rates[1]; Axis.PlanVY:=rates[2]; Axis.PlanVZ:=rates[3];
            Axis.PlanVA:=DEG(rates[4]); Axis.PlanVB:=DEG(rates[5]);
            Axis.PlanVC:=DEG(rates[6]);
        END_IF;
    ELSE
        Axis.PlanX:=tgtDeg[1]; Axis.PlanY:=tgtDeg[2]; Axis.PlanZ:=tgtDeg[3];
        Axis.PlanA:=tgtDeg[4]; Axis.PlanB:=tgtDeg[5]; Axis.PlanC:=tgtDeg[6];
        Axis.PlanVX:=0.0; Axis.PlanVY:=0.0; Axis.PlanVZ:=0.0;
        Axis.PlanVA:=0.0; Axis.PlanVB:=0.0; Axis.PlanVC:=0.0;
    END_IF;
END_IF;

(* ============================================================================
   第六步：完成 → 释放轴控制权
   ============================================================================ *)
IF running AND (phase=4) THEN
    running:=FALSE;
    Busy:=FALSE; InPos:=TRUE;
    Axis.Busy:=FALSE; Axis.ActiveType:=0; Axis.owner:=0; Axis.InPos:=TRUE;
END_IF;

END_FUNCTION_BLOCK
