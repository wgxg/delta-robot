(* ============================================================================
   功能块名称 : FB_LIN6（笛卡尔直线：位置 lerp + 姿态四元数 slerp）
   功能描述 : 对应 Python 黄金参考 test_delta6.py 的 FB_LIN6。
              · 上升沿 IK 预检，非法当周期 Rejected
              · 启动：起点按提升时刻 Cur 重取；q0/q1 半角四元数；
                total=hypot(平移mm, 100*短弧转角rad)
              · 每拍 _plan_pose(s)（位置线性，姿态短弧球面插值）+
                中心差分速度；规划姿态内部 rad、发布转度
              · 调度语义与 3DOF/FB_PTP6 逐行一致
   ============================================================================ *)

FUNCTION_BLOCK FB_LIN6
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
    fbChk : FB_DeltaKinematics6;   (* 上升沿 IK 预检用独立实例 *)

    (* 几何/轨迹状态（startDeg/tgtDeg 位置 mm、姿态度） *)
    startDeg : ARRAY[1..6] OF LREAL := (0.0,0.0,300.0,0.0,0.0,0.0);
    tgtDeg   : ARRAY[1..6] OF LREAL := (0.0,0.0,300.0,0.0,0.0,0.0);
    q0w,q0x,q0y,q0z : LREAL;
    q1w,q1x,q1y,q1z : LREAL;

    poseR  : ARRAY[1..6] OF LREAL;
    curAng : ARRAY[1..6] OF LREAL;
    pv, pr, p0v, p1v, rates : ARRAY[1..6] OF LREAL;

    s, sVel, sAcc : LREAL;
    maxSVel, maxSAcc, maxSJerk : LREAL;
    sStop, sRem : LREAL;
    total, dpos, drot : LREAL;
    sp0, sp1, ss : LREAL;
    phase : INT;
    geomOK : BOOL;
    k : INT;

    (* rpy_to_quat 启动临时量 *)
    ca,sa,cb,sb,cc,sc : LREAL;
    dotq : LREAL;

    (* slerp + quat_to_rpy 内联临时量（翻转只动 a1，不改持久 q1） *)
    d, a1w,a1x,a1y,a1z, th, sth, k0, k1, nrm : LREAL;
    qw,qx,qy,qz : LREAL;
    tmp, tmp2 : LREAL;

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
    fbChk(Mode:=3,
          R:=Axis.R, r:=Axis.r, L:=Axis.L, l:=Axis.l,
          Rac:=Axis.Rac, ra:=Axis.ra, La:=Axis.La, lb:=Axis.lb,
          Pose6:=poseR, Ref6:=curAng);
    geomOK:=fbChk.OK;

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
   第三步：统一启动（令牌 + 几何初始化；起点按提升时刻 Cur 重取）
   ============================================================================ *)
IF wantStart THEN
    Axis.cmdCount:=Axis.cmdCount+1;
    myId:=Axis.cmdCount;
    Axis.owner:=myId;
    Axis.ActiveType:=2;
    Axis.Busy:=TRUE;
    Axis.InPos:=FALSE;
    Axis.hasBuf:=FALSE;
    running:=TRUE; Busy:=TRUE; InPos:=FALSE;

    startDeg[1]:=Axis.CurX; startDeg[2]:=Axis.CurY; startDeg[3]:=Axis.CurZ;
    startDeg[4]:=Axis.CurA; startDeg[5]:=Axis.CurB; startDeg[6]:=Axis.CurC;

    (* q0 = rpy_to_quat(start 弧度姿态) *)
    ca:=COS(RAD(startDeg[4])*0.5); sa:=SIN(RAD(startDeg[4])*0.5);
    cb:=COS(RAD(startDeg[5])*0.5); sb:=SIN(RAD(startDeg[5])*0.5);
    cc:=COS(RAD(startDeg[6])*0.5); sc:=SIN(RAD(startDeg[6])*0.5);
    q0w:=cc*cb*ca+sc*sb*sa;
    q0x:=cc*cb*sa-sc*sb*ca;
    q0y:=cc*sb*ca+sc*cb*sa;
    q0z:=sc*cb*ca-cc*sb*sa;

    (* q1 = rpy_to_quat(tgt 弧度姿态) *)
    ca:=COS(RAD(tgtDeg[4])*0.5); sa:=SIN(RAD(tgtDeg[4])*0.5);
    cb:=COS(RAD(tgtDeg[5])*0.5); sb:=SIN(RAD(tgtDeg[5])*0.5);
    cc:=COS(RAD(tgtDeg[6])*0.5); sc:=SIN(RAD(tgtDeg[6])*0.5);
    q1w:=cc*cb*ca+sc*sb*sa;
    q1x:=cc*cb*sa-sc*sb*ca;
    q1y:=cc*sb*ca+sc*cb*sa;
    q1z:=sc*cb*ca-cc*sb*sa;

    dpos:=(tgtDeg[1]-startDeg[1])*(tgtDeg[1]-startDeg[1])
         +(tgtDeg[2]-startDeg[2])*(tgtDeg[2]-startDeg[2])
         +(tgtDeg[3]-startDeg[3])*(tgtDeg[3]-startDeg[3]);
    dpos:=SQRT(dpos);
    dotq:=q0w*q1w+q0x*q1x+q0y*q1y+q0z*q1z;
    IF dotq<0.0 THEN dotq:=-dotq; END_IF;
    IF dotq>1.0 THEN dotq:=1.0; END_IF;
    drot:=2.0*ACOS(dotq);

    total:=SQRT(dpos*dpos+(100.0*drot)*(100.0*drot));

    IF total<1.0E-4 THEN
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
   第五步：轨迹推进（lerp + slerp，三次求值做主位姿与中心差分）
   ============================================================================ *)
IF running AND (phase=1 OR phase=2 OR phase=3) THEN

    (* 梯形/S 曲线 profile *)
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
        (* --- 主位姿 ss=s --- *)
        ss:=s;
        pv[1]:=startDeg[1]+(tgtDeg[1]-startDeg[1])*ss;
        pv[2]:=startDeg[2]+(tgtDeg[2]-startDeg[2])*ss;
        pv[3]:=startDeg[3]+(tgtDeg[3]-startDeg[3])*ss;
        d:=q0w*q1w+q0x*q1x+q0y*q1y+q0z*q1z;
        a1w:=q1w; a1x:=q1x; a1y:=q1y; a1z:=q1z;
        IF d<0.0 THEN
            a1w:=-a1w; a1x:=-a1x; a1y:=-a1y; a1z:=-a1z; d:=-d;
        END_IF;
        IF d>0.9995 THEN
            qw:=q0w+ss*(a1w-q0w); qx:=q0x+ss*(a1x-q0x);
            qy:=q0y+ss*(a1y-q0y); qz:=q0z+ss*(a1z-q0z);
            nrm:=SQRT(qw*qw+qx*qx+qy*qy+qz*qz);
            qw:=qw/nrm; qx:=qx/nrm; qy:=qy/nrm; qz:=qz/nrm;
        ELSE
            th:=ACOS(d); sth:=SIN(th);
            k0:=SIN((1.0-ss)*th)/sth;
            k1:=SIN(ss*th)/sth;
            qw:=k0*q0w+k1*a1w; qx:=k0*q0x+k1*a1x;
            qy:=k0*q0y+k1*a1y; qz:=k0*q0z+k1*a1z;
        END_IF;
        pv[4]:=ATAN2(2.0*(qw*qx+qy*qz), 1.0-2.0*(qx*qx+qy*qy));
        tmp2:=2.0*(qw*qy-qz*qx);
        IF tmp2>1.0 THEN tmp2:=1.0; END_IF;
        IF tmp2<-1.0 THEN tmp2:=-1.0; END_IF;
        pv[5]:=ASIN(tmp2);
        pv[6]:=ATAN2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz));
        FOR k:=1 TO 6 DO pr[k]:=pv[k]; END_FOR;

        (* --- s0=min(s,1) --- *)
        sp0:=s;
        IF sp0>1.0 THEN sp0:=1.0; END_IF;
        ss:=sp0;
        pv[1]:=startDeg[1]+(tgtDeg[1]-startDeg[1])*ss;
        pv[2]:=startDeg[2]+(tgtDeg[2]-startDeg[2])*ss;
        pv[3]:=startDeg[3]+(tgtDeg[3]-startDeg[3])*ss;
        d:=q0w*q1w+q0x*q1x+q0y*q1y+q0z*q1z;
        a1w:=q1w; a1x:=q1x; a1y:=q1y; a1z:=q1z;
        IF d<0.0 THEN
            a1w:=-a1w; a1x:=-a1x; a1y:=-a1y; a1z:=-a1z; d:=-d;
        END_IF;
        IF d>0.9995 THEN
            qw:=q0w+ss*(a1w-q0w); qx:=q0x+ss*(a1x-q0x);
            qy:=q0y+ss*(a1y-q0y); qz:=q0z+ss*(a1z-q0z);
            nrm:=SQRT(qw*qw+qx*qx+qy*qy+qz*qz);
            qw:=qw/nrm; qx:=qx/nrm; qy:=qy/nrm; qz:=qz/nrm;
        ELSE
            th:=ACOS(d); sth:=SIN(th);
            k0:=SIN((1.0-ss)*th)/sth;
            k1:=SIN(ss*th)/sth;
            qw:=k0*q0w+k1*a1w; qx:=k0*q0x+k1*a1x;
            qy:=k0*q0y+k1*a1y; qz:=k0*q0z+k1*a1z;
        END_IF;
        pv[4]:=ATAN2(2.0*(qw*qx+qy*qz), 1.0-2.0*(qx*qx+qy*qy));
        tmp2:=2.0*(qw*qy-qz*qx);
        IF tmp2>1.0 THEN tmp2:=1.0; END_IF;
        IF tmp2<-1.0 THEN tmp2:=-1.0; END_IF;
        pv[5]:=ASIN(tmp2);
        pv[6]:=ATAN2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz));
        FOR k:=1 TO 6 DO p0v[k]:=pv[k]; END_FOR;

        (* --- s1=min(s+1e-6,1) --- *)
        sp1:=s+1.0E-6;
        IF sp1>1.0 THEN sp1:=1.0; END_IF;
        IF sp1<=sp0 THEN
            FOR k:=1 TO 6 DO rates[k]:=0.0; END_FOR;
        ELSE
            ss:=sp1;
            pv[1]:=startDeg[1]+(tgtDeg[1]-startDeg[1])*ss;
            pv[2]:=startDeg[2]+(tgtDeg[2]-startDeg[2])*ss;
            pv[3]:=startDeg[3]+(tgtDeg[3]-startDeg[3])*ss;
            d:=q0w*q1w+q0x*q1x+q0y*q1y+q0z*q1z;
            a1w:=q1w; a1x:=q1x; a1y:=q1y; a1z:=q1z;
            IF d<0.0 THEN
                a1w:=-a1w; a1x:=-a1x; a1y:=-a1y; a1z:=-a1z; d:=-d;
            END_IF;
            IF d>0.9995 THEN
                qw:=q0w+ss*(a1w-q0w); qx:=q0x+ss*(a1x-q0x);
                qy:=q0y+ss*(a1y-q0y); qz:=q0z+ss*(a1z-q0z);
                nrm:=SQRT(qw*qw+qx*qx+qy*qy+qz*qz);
                qw:=qw/nrm; qx:=qx/nrm; qy:=qy/nrm; qz:=qz/nrm;
            ELSE
                th:=ACOS(d); sth:=SIN(th);
                k0:=SIN((1.0-ss)*th)/sth;
                k1:=SIN(ss*th)/sth;
                qw:=k0*q0w+k1*a1w; qx:=k0*q0x+k1*a1x;
                qy:=k0*q0y+k1*a1y; qz:=k0*q0z+k1*a1z;
            END_IF;
            pv[4]:=ATAN2(2.0*(qw*qx+qy*qz), 1.0-2.0*(qx*qx+qy*qy));
            tmp2:=2.0*(qw*qy-qz*qx);
            IF tmp2>1.0 THEN tmp2:=1.0; END_IF;
            IF tmp2<-1.0 THEN tmp2:=-1.0; END_IF;
            pv[5]:=ASIN(tmp2);
            pv[6]:=ATAN2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz));
            FOR k:=1 TO 6 DO
                p1v[k]:=pv[k];
                rates[k]:=(p1v[k]-p0v[k])/1.0E-6*sVel;
            END_FOR;
        END_IF;

        Axis.PlanX:=pr[1]; Axis.PlanY:=pr[2]; Axis.PlanZ:=pr[3];
        Axis.PlanA:=DEG(pr[4]); Axis.PlanB:=DEG(pr[5]); Axis.PlanC:=DEG(pr[6]);
        Axis.PlanVX:=rates[1]; Axis.PlanVY:=rates[2]; Axis.PlanVZ:=rates[3];
        Axis.PlanVA:=DEG(rates[4]); Axis.PlanVB:=DEG(rates[5]);
        Axis.PlanVC:=DEG(rates[6]);
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
