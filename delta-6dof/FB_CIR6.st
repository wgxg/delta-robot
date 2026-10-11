(* ============================================================================
   功能块名称 : FB_CIR6（笛卡尔圆弧：位置圆弧 + 姿态沿弧 slerp）
   功能描述 : 对应 Python 黄金参考 test_delta6.py 的 FB_CIR6。
              · CirMode=0 圆心式（Aux=圆心）；CirMode=1 三点式（Aux=途经点）
              · 上升沿 IK + 圆弧几何双预检，非法当周期 Rejected
              · 缓存提升时几何必须按提升时刻 Cur【重建】；
                arc_len = radius*|thetaSpan|（不得把布尔当弧长）
              · 每拍圆心+radius(cos·u+sin·v) 求位置，姿态 slerp；
                中心差分速度；规划姿态内部 rad、发布转度
   ============================================================================ *)

FUNCTION_BLOCK FB_CIR6
VAR_INPUT
    Execute      : BOOL;
    PositionX, PositionY, PositionZ : LREAL;
    AuxX, AuxY, AuxZ : LREAL;
    CirMode      : INT;        (* 0=圆心式 1=三点式 *)
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
    fbChk : FB_DeltaKinematics6;   (* 上升沿 IK 预检 *)

    (* 目标/辅助（位置 mm、姿态度） *)
    tgtDeg   : ARRAY[1..6] OF LREAL := (0.0,0.0,300.0,0.0,0.0,0.0);
    startDeg : ARRAY[1..6] OF LREAL := (0.0,0.0,300.0,0.0,0.0,0.0);
    aux      : ARRAY[1..3] OF LREAL;
    cirMode  : INT;

    (* 成功几何（仅 build 成功时整体更新，失败保持旧值） *)
    cx,cy,cz, radius, thetaSpan : LREAL;
    uu : ARRAY[1..3] OF LREAL;
    vv : ARRAY[1..3] OF LREAL;

    q0w,q0x,q0y,q0z : LREAL;
    q1w,q1x,q1y,q1z : LREAL;

    poseR  : ARRAY[1..6] OF LREAL;
    curAng : ARRAY[1..6] OF LREAL;
    pv, pr, p0v, p1v, rates : ARRAY[1..6] OF LREAL;

    s, sVel, sAcc : LREAL;
    maxSVel, maxSAcc, maxSJerk : LREAL;
    sStop, sRem : LREAL;
    total, drot, arcLen : LREAL;
    sp0, sp1, ss : LREAL;
    phase : INT;
    geomOK, ikOK, buildOK : BOOL;
    k : INT;

    (* _build_geom 局部几何（成功后拷入持久几何） *)
    lcx,lcy,lcz, lr, lspan : LREAL;
    lu : ARRAY[1..3] OF LREAL;
    lv : ARRAY[1..3] OF LREAL;
    sx,sy,sz, ex,ey,ez : LREAL;
    sm : ARRAY[1..3] OF LREAL;
    se : ARRAY[1..3] OF LREAL;
    nn : ARRAY[1..3] OF LREAL;
    cs : ARRAY[1..3] OF LREAL;
    ce : ARRAY[1..3] OF LREAL;
    cm : ARRAY[1..3] OF LREAL;
    nlen, smlen, eu, ev, aa, bb, te, tm : LREAL;

    (* rpy_to_quat / slerp / quat_to_rpy 临时量 *)
    ca,sa,cb,sb,cc,sc : LREAL;
    dotq : LREAL;
    d, a1w,a1x,a1y,a1z, th, sth, k0, k1, nrm : LREAL;
    qw,qx,qy,qz : LREAL;
    theta, ct, stt : LREAL;
    tmp2 : LREAL;

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
   第一步：上升沿仲裁（IK + 圆弧几何双预检）
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
    aux[1]:=AuxX; aux[2]:=AuxY; aux[3]:=AuxZ;
    cirMode:=CirMode;

    FOR k:=1 TO 6 DO curAng[k]:=Axis.CurTheta[k]; END_FOR;
    poseR[1]:=PositionX; poseR[2]:=PositionY; poseR[3]:=PositionZ;
    poseR[4]:=RAD(PositionA); poseR[5]:=RAD(PositionB); poseR[6]:=RAD(PositionC);
    fbChk(Mode:=3,
          R:=Axis.R, r:=Axis.r, L:=Axis.L, l:=Axis.l,
          Rac:=Axis.Rac, ra:=Axis.ra, La:=Axis.La, lb:=Axis.lb,
          Pose6:=poseR, Ref6:=curAng);
    ikOK:=fbChk.OK;

    (* --- _build_geom（上升沿预检；ikOK 失败时短路不建几何） --- *)
    buildOK:=TRUE;
    IF ikOK THEN
        sx:=Axis.CurX; sy:=Axis.CurY; sz:=Axis.CurZ;
        ex:=tgtDeg[1]; ey:=tgtDeg[2]; ez:=tgtDeg[3];

        IF cirMode=0 THEN
            lcx:=aux[1]; lcy:=aux[2]; lcz:=aux[3];
        ELSE
            sm[1]:=aux[1]-sx; sm[2]:=aux[2]-sy; sm[3]:=aux[3]-sz;
            se[1]:=ex-sx; se[2]:=ey-sy; se[3]:=ez-sz;
            nn[1]:=sm[2]*se[3]-sm[3]*se[2];
            nn[2]:=sm[3]*se[1]-sm[1]*se[3];
            nn[3]:=sm[1]*se[2]-sm[2]*se[1];
            nlen:=SQRT(nn[1]*nn[1]+nn[2]*nn[2]+nn[3]*nn[3]);
            IF nlen<1.0E-6 THEN buildOK:=FALSE; END_IF;
            IF buildOK THEN
                nn[1]:=nn[1]/nlen; nn[2]:=nn[2]/nlen; nn[3]:=nn[3]/nlen;
                smlen:=SQRT(sm[1]*sm[1]+sm[2]*sm[2]+sm[3]*sm[3]);
                lu[1]:=sm[1]/smlen; lu[2]:=sm[2]/smlen; lu[3]:=sm[3]/smlen;
                lv[1]:=nn[2]*lu[3]-nn[3]*lu[2];
                lv[2]:=nn[3]*lu[1]-nn[1]*lu[3];
                lv[3]:=nn[1]*lu[2]-nn[2]*lu[1];
                eu:=se[1]*lu[1]+se[2]*lu[2]+se[3]*lu[3];
                ev:=se[1]*lv[1]+se[2]*lv[2]+se[3]*lv[3];
                aa:=smlen/2.0;
                IF ABS(ev)<1.0E-6 THEN buildOK:=FALSE; END_IF;
                IF buildOK THEN
                    bb:=(eu*eu+ev*ev-2.0*aa*eu)/(2.0*ev);
                    lcx:=sx+aa*lu[1]+bb*lv[1];
                    lcy:=sy+aa*lu[2]+bb*lv[2];
                    lcz:=sz+aa*lu[3]+bb*lv[3];
                END_IF;
            END_IF;
        END_IF;

        IF buildOK THEN
            cs[1]:=sx-lcx; cs[2]:=sy-lcy; cs[3]:=sz-lcz;
            lr:=SQRT(cs[1]*cs[1]+cs[2]*cs[2]+cs[3]*cs[3]);
            ce[1]:=ex-lcx; ce[2]:=ey-lcy; ce[3]:=ez-lcz;
            IF lr<1.0E-6 THEN buildOK:=FALSE; END_IF;
        END_IF;
        IF buildOK THEN
            lu[1]:=cs[1]/lr; lu[2]:=cs[2]/lr; lu[3]:=cs[3]/lr;
            IF cirMode=0 THEN
                nn[1]:=cs[2]*ce[3]-cs[3]*ce[2];
                nn[2]:=cs[3]*ce[1]-cs[1]*ce[3];
                nn[3]:=cs[1]*ce[2]-cs[2]*ce[1];
                nlen:=SQRT(nn[1]*nn[1]+nn[2]*nn[2]+nn[3]*nn[3]);
                IF nlen<1.0E-6 THEN buildOK:=FALSE; END_IF;
                IF buildOK THEN
                    nn[1]:=nn[1]/nlen; nn[2]:=nn[2]/nlen; nn[3]:=nn[3]/nlen;
                END_IF;
            END_IF;
        END_IF;
        IF buildOK THEN
            lv[1]:=nn[2]*lu[3]-nn[3]*lu[2];
            lv[2]:=nn[3]*lu[1]-nn[1]*lu[3];
            lv[3]:=nn[1]*lu[2]-nn[2]*lu[1];
            eu:=ce[1]*lu[1]+ce[2]*lu[2]+ce[3]*lu[3];
            ev:=ce[1]*lv[1]+ce[2]*lv[2]+ce[3]*lv[3];
            te:=ATAN2(ev,eu);
            IF cirMode=1 THEN
                cm[1]:=aux[1]-lcx; cm[2]:=aux[2]-lcy; cm[3]:=aux[3]-lcz;
                tm:=ATAN2(cm[1]*lv[1]+cm[2]*lv[2]+cm[3]*lv[3],
                          cm[1]*lu[1]+cm[2]*lu[2]+cm[3]*lu[3]);
                IF te<0.0 THEN te:=te+6.283185307179586; END_IF;
                IF tm<0.0 THEN tm:=tm+6.283185307179586; END_IF;
                IF NOT (tm>0.0 AND tm<te) THEN
                    te:=te-6.283185307179586;
                END_IF;
            ELSE
                IF te<0.0 THEN te:=te+6.283185307179586; END_IF;
                IF te>3.141592653589793 THEN te:=te-6.283185307179586; END_IF;
            END_IF;
            lspan:=te;
            IF lr*ABS(lspan)<1.0E-4 THEN buildOK:=FALSE; END_IF;
        END_IF;
    END_IF;
    geomOK:=ikOK AND buildOK;
    IF geomOK THEN
        cx:=lcx; cy:=lcy; cz:=lcz; radius:=lr; thetaSpan:=lspan;
        FOR k:=1 TO 3 DO uu[k]:=lu[k]; vv[k]:=lv[k]; END_FOR;
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
   第三步：统一启动（几何按提升时刻 Cur 重建）
   ============================================================================ *)
IF wantStart THEN
    Axis.cmdCount:=Axis.cmdCount+1;
    myId:=Axis.cmdCount;
    Axis.owner:=myId;
    Axis.ActiveType:=3;
    Axis.Busy:=TRUE;
    Axis.InPos:=FALSE;
    Axis.hasBuf:=FALSE;
    running:=TRUE; Busy:=TRUE; InPos:=FALSE;

    (* --- _build_geom 重建（起点=提升时刻 Cur） --- *)
    buildOK:=TRUE;
    sx:=Axis.CurX; sy:=Axis.CurY; sz:=Axis.CurZ;
    ex:=tgtDeg[1]; ey:=tgtDeg[2]; ez:=tgtDeg[3];

    IF cirMode=0 THEN
        lcx:=aux[1]; lcy:=aux[2]; lcz:=aux[3];
    ELSE
        sm[1]:=aux[1]-sx; sm[2]:=aux[2]-sy; sm[3]:=aux[3]-sz;
        se[1]:=ex-sx; se[2]:=ey-sy; se[3]:=ez-sz;
        nn[1]:=sm[2]*se[3]-sm[3]*se[2];
        nn[2]:=sm[3]*se[1]-sm[1]*se[3];
        nn[3]:=sm[1]*se[2]-sm[2]*se[1];
        nlen:=SQRT(nn[1]*nn[1]+nn[2]*nn[2]+nn[3]*nn[3]);
        IF nlen<1.0E-6 THEN buildOK:=FALSE; END_IF;
        IF buildOK THEN
            nn[1]:=nn[1]/nlen; nn[2]:=nn[2]/nlen; nn[3]:=nn[3]/nlen;
            smlen:=SQRT(sm[1]*sm[1]+sm[2]*sm[2]+sm[3]*sm[3]);
            lu[1]:=sm[1]/smlen; lu[2]:=sm[2]/smlen; lu[3]:=sm[3]/smlen;
            lv[1]:=nn[2]*lu[3]-nn[3]*lu[2];
            lv[2]:=nn[3]*lu[1]-nn[1]*lu[3];
            lv[3]:=nn[1]*lu[2]-nn[2]*lu[1];
            eu:=se[1]*lu[1]+se[2]*lu[2]+se[3]*lu[3];
            ev:=se[1]*lv[1]+se[2]*lv[2]+se[3]*lv[3];
            aa:=smlen/2.0;
            IF ABS(ev)<1.0E-6 THEN buildOK:=FALSE; END_IF;
            IF buildOK THEN
                bb:=(eu*eu+ev*ev-2.0*aa*eu)/(2.0*ev);
                lcx:=sx+aa*lu[1]+bb*lv[1];
                lcy:=sy+aa*lu[2]+bb*lv[2];
                lcz:=sz+aa*lu[3]+bb*lv[3];
            END_IF;
        END_IF;
    END_IF;

    IF buildOK THEN
        cs[1]:=sx-lcx; cs[2]:=sy-lcy; cs[3]:=sz-lcz;
        lr:=SQRT(cs[1]*cs[1]+cs[2]*cs[2]+cs[3]*cs[3]);
        ce[1]:=ex-lcx; ce[2]:=ey-lcy; ce[3]:=ez-lcz;
        IF lr<1.0E-6 THEN buildOK:=FALSE; END_IF;
    END_IF;
    IF buildOK THEN
        lu[1]:=cs[1]/lr; lu[2]:=cs[2]/lr; lu[3]:=cs[3]/lr;
        IF cirMode=0 THEN
            nn[1]:=cs[2]*ce[3]-cs[3]*ce[2];
            nn[2]:=cs[3]*ce[1]-cs[1]*ce[3];
            nn[3]:=cs[1]*ce[2]-cs[2]*ce[1];
            nlen:=SQRT(nn[1]*nn[1]+nn[2]*nn[2]+nn[3]*nn[3]);
            IF nlen<1.0E-6 THEN buildOK:=FALSE; END_IF;
            IF buildOK THEN
                nn[1]:=nn[1]/nlen; nn[2]:=nn[2]/nlen; nn[3]:=nn[3]/nlen;
            END_IF;
        END_IF;
    END_IF;
    IF buildOK THEN
        lv[1]:=nn[2]*lu[3]-nn[3]*lu[2];
        lv[2]:=nn[3]*lu[1]-nn[1]*lu[3];
        lv[3]:=nn[1]*lu[2]-nn[2]*lu[1];
        eu:=ce[1]*lu[1]+ce[2]*lu[2]+ce[3]*lu[3];
        ev:=ce[1]*lv[1]+ce[2]*lv[2]+ce[3]*lv[3];
        te:=ATAN2(ev,eu);
        IF cirMode=1 THEN
            cm[1]:=aux[1]-lcx; cm[2]:=aux[2]-lcy; cm[3]:=aux[3]-lcz;
            tm:=ATAN2(cm[1]*lv[1]+cm[2]*lv[2]+cm[3]*lv[3],
                      cm[1]*lu[1]+cm[2]*lu[2]+cm[3]*lu[3]);
            IF te<0.0 THEN te:=te+6.283185307179586; END_IF;
            IF tm<0.0 THEN tm:=tm+6.283185307179586; END_IF;
            IF NOT (tm>0.0 AND tm<te) THEN
                te:=te-6.283185307179586;
            END_IF;
        ELSE
            IF te<0.0 THEN te:=te+6.283185307179586; END_IF;
            IF te>3.141592653589793 THEN te:=te-6.283185307179586; END_IF;
        END_IF;
        lspan:=te;
        IF lr*ABS(lspan)<1.0E-4 THEN buildOK:=FALSE; END_IF;
    END_IF;

    IF NOT buildOK THEN
        Axis.PlanX:=tgtDeg[1]; Axis.PlanY:=tgtDeg[2]; Axis.PlanZ:=tgtDeg[3];
        Axis.PlanA:=tgtDeg[4]; Axis.PlanB:=tgtDeg[5]; Axis.PlanC:=tgtDeg[6];
        Axis.PlanVX:=0.0; Axis.PlanVY:=0.0; Axis.PlanVZ:=0.0;
        Axis.PlanVA:=0.0; Axis.PlanVB:=0.0; Axis.PlanVC:=0.0;
        s:=1.0; phase:=4;
        arcLen:=0.0;
    ELSE
        cx:=lcx; cy:=lcy; cz:=lcz; radius:=lr; thetaSpan:=lspan;
        FOR k:=1 TO 3 DO uu[k]:=lu[k]; vv[k]:=lv[k]; END_FOR;
        arcLen:=radius*ABS(thetaSpan);
    END_IF;

    startDeg[1]:=Axis.CurX; startDeg[2]:=Axis.CurY; startDeg[3]:=Axis.CurZ;
    startDeg[4]:=Axis.CurA; startDeg[5]:=Axis.CurB; startDeg[6]:=Axis.CurC;

    ca:=COS(RAD(startDeg[4])*0.5); sa:=SIN(RAD(startDeg[4])*0.5);
    cb:=COS(RAD(startDeg[5])*0.5); sb:=SIN(RAD(startDeg[5])*0.5);
    cc:=COS(RAD(startDeg[6])*0.5); sc:=SIN(RAD(startDeg[6])*0.5);
    q0w:=cc*cb*ca+sc*sb*sa;
    q0x:=cc*cb*sa-sc*sb*ca;
    q0y:=cc*sb*ca+sc*cb*sa;
    q0z:=sc*cb*ca-cc*sb*sa;

    ca:=COS(RAD(tgtDeg[4])*0.5); sa:=SIN(RAD(tgtDeg[4])*0.5);
    cb:=COS(RAD(tgtDeg[5])*0.5); sb:=SIN(RAD(tgtDeg[5])*0.5);
    cc:=COS(RAD(tgtDeg[6])*0.5); sc:=SIN(RAD(tgtDeg[6])*0.5);
    q1w:=cc*cb*ca+sc*sb*sa;
    q1x:=cc*cb*sa-sc*sb*ca;
    q1y:=cc*sb*ca+sc*cb*sa;
    q1z:=sc*cb*ca-cc*sb*sa;

    dotq:=q0w*q1w+q0x*q1x+q0y*q1y+q0z*q1z;
    IF dotq<0.0 THEN dotq:=-dotq; END_IF;
    IF dotq>1.0 THEN dotq:=1.0; END_IF;
    drot:=2.0*ACOS(dotq);

    total:=SQRT(arcLen*arcLen+(100.0*drot)*(100.0*drot));
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
   第五步：轨迹推进（圆弧 + slerp，三次求值做主位姿与中心差分）
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
        theta:=thetaSpan*ss;
        ct:=COS(theta); stt:=SIN(theta);
        pv[1]:=cx+radius*(ct*uu[1]+stt*vv[1]);
        pv[2]:=cy+radius*(ct*uu[2]+stt*vv[2]);
        pv[3]:=cz+radius*(ct*uu[3]+stt*vv[3]);
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
        theta:=thetaSpan*ss;
        ct:=COS(theta); stt:=SIN(theta);
        pv[1]:=cx+radius*(ct*uu[1]+stt*vv[1]);
        pv[2]:=cy+radius*(ct*uu[2]+stt*vv[2]);
        pv[3]:=cz+radius*(ct*uu[3]+stt*vv[3]);
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
            theta:=thetaSpan*ss;
            ct:=COS(theta); stt:=SIN(theta);
            pv[1]:=cx+radius*(ct*uu[1]+stt*vv[1]);
            pv[2]:=cy+radius*(ct*uu[2]+stt*vv[2]);
            pv[3]:=cz+radius*(ct*uu[3]+stt*vv[3]);
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
