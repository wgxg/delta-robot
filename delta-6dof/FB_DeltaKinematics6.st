(* ============================================================================
   文件名称 : FB_DeltaKinematics6.st
   功能描述 : Delta 6DOF 纯运动学计算件（无状态、多模式），逐行对应 Python
              黄金参考 kinematics6.py：
                Mode=1  fk_track  列归一化 + Levenberg-Marquardt + 残差严格
                        单调线搜索（控制环温启动，装配连续，见 DESIGN 3.3）
                Mode=2  fk        纯牛顿（列归一化 + 步长限幅，冷启动）
                Mode=3  ik        六链全闭式逆解（多解取距参考角最近）
                Mode=4  ik_vel    解析关节速度前馈
              持久状态（tar/guess/cur/cold）由 FB_DeltaCore6 持有并按
              test_delta6.py 的 FB_DeltaKinematics6.__call__ 顺序编排。
   同文件私有辅助块：FB_Solve6（部分主元高斯消元，对应 _solve6）。
   全部 LREAL 双精度，与 Python float 运算顺序逐字对齐。
   ============================================================================ *)


FUNCTION_BLOCK FB_Solve6
VAR_INPUT
    A : ARRAY[1..6,1..6] OF LREAL;
    b : ARRAY[1..6] OF LREAL;
END_VAR
VAR_OUTPUT
    X  : ARRAY[1..6] OF LREAL;
    OK : BOOL;
END_VAR
VAR
    M   : ARRAY[1..6,1..7] OF LREAL;
    col, rr, c2, piv : INT;
    pmax, pv, fct, ss : LREAL;
END_VAR

(* 增广矩阵 M = [A | b] *)
FOR rr:=1 TO 6 DO
    FOR c2:=1 TO 6 DO
        M[rr,c2]:=A[rr,c2];
    END_FOR;
    M[rr,7]:=b[rr];
END_FOR;

OK:=TRUE;
FOR col:=1 TO 6 DO
    piv:=col;
    pmax:=ABS(M[col,col]);
    FOR rr:=col+1 TO 6 DO
        IF ABS(M[rr,col]) > pmax THEN
            pmax:=ABS(M[rr,col]);
            piv:=rr;
        END_IF;
    END_FOR;
    IF pmax < 1.0E-12 THEN
        OK:=FALSE;
        EXIT;
    END_IF;
    IF piv <> col THEN
        FOR c2:=1 TO 7 DO
            ss:=M[col,c2]; M[col,c2]:=M[piv,c2]; M[piv,c2]:=ss;
        END_FOR;
    END_IF;
    pv:=M[col,col];
    FOR rr:=col+1 TO 6 DO
        fct:=M[rr,col]/pv;
        IF fct <> 0.0 THEN
            FOR c2:=col TO 7 DO
                M[rr,c2]:=M[rr,c2]-fct*M[col,c2];
            END_FOR;
        END_IF;
    END_FOR;
END_FOR;

IF OK THEN
    FOR rr:=6 TO 1 BY -1 DO
        ss:=M[rr,7];
        FOR c2:=rr+1 TO 6 DO
            ss:=ss-M[rr,c2]*X[c2];
        END_FOR;
        X[rr]:=ss/M[rr,rr];
    END_FOR;
END_IF;

END_FUNCTION_BLOCK


(* ========================================================================== *)
FUNCTION_BLOCK FB_DeltaKinematics6
VAR_INPUT
    Mode          : INT;       (* 1=FK_Track 2=FK_Cold 3=IK 4=IK_Vel        *)
    R,r,L,l       : LREAL;     (* 主链参数                                   *)
    Rac,ra,La,lb  : LREAL;     (* 副链参数                                   *)
    Ang6          : ARRAY[1..6] OF LREAL;   (* 电机角（FK/ik_vel 输入）rad *)
    Guess6        : ARRAY[1..6] OF LREAL;   (* FK 初值位姿 rad             *)
    Pose6         : ARRAY[1..6] OF LREAL;   (* 位姿 rad（IK/ik_vel 输入） *)
    Ref6          : ARRAY[1..6] OF LREAL;   (* IK 参考角 rad              *)
    Twist6        : ARRAY[1..6] OF LREAL;   (* 位姿速度 mm/s,rad/s        *)
END_VAR
VAR_OUTPUT
    OK            : BOOL;
    OutVec6       : ARRAY[1..6] OF LREAL;   (* pose rad / 角度 rad / 速度 *)
END_VAR
VAR
    (* 方位角 cos/sin：每次调用由角度重算（不得硬编码 -0.5/±√3/2：
       cos(4π/3)=-0.5000000000000004、sin(4π/3)=-0.8660254037844385 与
       十进制常数末位不同，对称位形下会破坏残差严格抵消，逐拍对拍发散） *)
    CPH : ARRAY[1..3] OF LREAL;
    SPH : ARRAY[1..3] OF LREAL;
    PI2 : LREAL := 6.283185307179586;

    sol : FB_Solve6;

    (* 肘点与链参数 *)
    Eb   : ARRAY[1..6,1..3] OF LREAL;
    radP, armV, rodV, baseR, refV : LREAL;
    k, j3, ki, it, tries, rr, c2 : INT;
    ce, se, er, cph, sph : LREAL;

    (* rot_partials 内联 *)
    ca,sa,cb,sb,cc,sc : LREAL;
    lax,lay,laz : LREAL;
    a1x,a1y,a1z,a2x,a2y,a2z : LREAL;
    rax,ray,raz : LREAL;
    t1x,t1y,t1z : LREAL;
    dax,day,daz,dbx,dby,dbz,dcx,dcy,dcz : LREAL;

    (* 残差/雅可比 *)
    x, xn : ARRAY[1..6] OF LREAL;
    J     : ARRAY[1..6,1..6] OF LREAL;
    fv,f1 : ARRAY[1..6] OF LREAL;
    cn,Jn : ARRAY[1..6,1..6] OF LREAL;
    JtJ,Mm: ARRAY[1..6,1..6] OF LREAL;
    Jtf   : ARRAY[1..6] OF LREAL;
    px,py,pz,a4,b4,c4 : LREAL;
    ax,ay,az,exv,eyv,ezv : LREAL;
    dxv,dyv,dzv,dist,err,maxerr : LREAL;
    dx6   : ARRAY[1..6] OF LREAL;
    step, tmp : LREAL;
    fkDone : BOOL;
    ssum : LREAL;

    (* LM *)
    lam, err0, e1 : LREAL;
    (* Neumaier 补偿求和（对应 CPython 内建 sum 的 float 部分和算法；
       JtJ/Jtf/err0/e1 四处必须与 Python 逐位一致，朴素左结合在近奇异
       姿态列会差 1ULP 并被病态矩阵放大，见 DESIGN 3.3） *)
    nsc, ntv, ntt : LREAL;
    accepted : BOOL;

    (* IK 单链 *)
    u,v,w,m2,mV,cv,xx,alpha,dd,tw,bestDD,thOut,dad : LREAL;
    found, ikAll : BOOL;

    (* 前馈 *)
    vx,vy,vz,wa,wb,wc : LREAL;
    wx,wy,wz,num,den,thp : LREAL;
END_VAR


(* ###########################################################################
   公共预处理：方位角 cos/sin + 6 个肘点，与 Python fk/fk_track 头部一致
   （PHI=0/2π/3/4π/3；角度字面量必须与 math.pi 运算结果同 double）
   ########################################################################### *)
CPH[1]:=1.0; SPH[1]:=0.0;
CPH[2]:=COS(2.0943951023931953); SPH[2]:=SIN(2.0943951023931953);
CPH[3]:=COS(4.1887902047863905); SPH[3]:=SIN(4.1887902047863905);

FOR k:=1 TO 3 DO
    j3:=k+3;
    cph:=CPH[k]; sph:=SPH[k];

    ce:=COS(Ang6[k]); se:=SIN(Ang6[k]);
    er:=R+L*ce;
    Eb[k,1]:=er*cph; Eb[k,2]:=er*sph; Eb[k,3]:=L*se;

    ce:=COS(Ang6[j3]); se:=SIN(Ang6[j3]);
    er:=Rac+La*ce;
    Eb[j3,1]:=er*cph; Eb[j3,2]:=er*sph; Eb[j3,3]:=La*se;
END_FOR;

OK:=FALSE;

IF Mode=1 OR Mode=2 THEN

    (* =======================================================================
       FK 主体：x=Guess6；Mode=2 纯牛顿(60次,步长限幅)；
                Mode=1 LM 跟踪(40次,λ自适应,残差严格单调)
       ======================================================================= *)
    FOR k:=1 TO 6 DO
        x[k]:=Guess6[k];
    END_FOR;

    IF Mode=2 THEN
        (* ---------------- 纯牛顿 fk() ---------------- *)
        fkDone:=FALSE;
        FOR it:=1 TO 60 DO
            (* 组装残差与雅可比 *)
            px:=x[1]; py:=x[2]; pz:=x[3]; a4:=x[4]; b4:=x[5]; c4:=x[6];
            ca:=COS(a4); sa:=SIN(a4); cb:=COS(b4); sb:=SIN(b4);
            cc:=COS(c4); sc:=SIN(c4);
            maxerr:=0.0;
            FOR k:=1 TO 6 DO
                ki:=((k-1) MOD 3)+1;
                IF k<=3 THEN radP:=r; ELSE radP:=ra; END_IF;
                IF k<=3 THEN rodV:=l; ELSE rodV:=lb; END_IF;
                lax:=radP*CPH[ki]; lay:=radP*SPH[ki]; laz:=0.0;
                (* rot_partials *)
                a1x:=lax; a1y:=ca*lay-sa*laz; a1z:=sa*lay+ca*laz;
                a2x:=cb*a1x+sb*a1z; a2y:=a1y; a2z:=-sb*a1x+cb*a1z;
                rax:=cc*a2x-sc*a2y; ray:=sc*a2x+cc*a2y; raz:=a2z;
                t1x:=sb*a1y; t1y:=-a1z; t1z:=cb*a1y;
                dax:=cc*t1x-sc*t1y; day:=sc*t1x+cc*t1y; daz:=t1z;
                dbx:=cc*a2z; dby:=sc*a2z; dbz:=-a2x;
                dcx:=-ray; dcy:=rax; dcz:=0.0;
                ax:=px+rax; ay:=py+ray; az:=pz+raz;
                exv:=Eb[k,1]; eyv:=Eb[k,2]; ezv:=Eb[k,3];
                dxv:=ax-exv; dyv:=ay-eyv; dzv:=az-ezv;
                dist:=SQRT(dxv*dxv+dyv*dyv+dzv*dzv);
                err:=ABS(dist-rodV);
                IF err>maxerr THEN maxerr:=err; END_IF;
                fv[k]:=dxv*dxv+dyv*dyv+dzv*dzv-rodV*rodV;
                J[k,1]:=2.0*dxv; J[k,2]:=2.0*dyv; J[k,3]:=2.0*dzv;
                J[k,4]:=2.0*(dxv*dax+dyv*day+dzv*daz);
                J[k,5]:=2.0*(dxv*dbx+dyv*dby+dzv*dbz);
                J[k,6]:=2.0*(dxv*dcx+dyv*dcy+dzv*dcz);
            END_FOR;

            IF maxerr < 1.0E-8 THEN
                FOR k:=1 TO 6 DO OutVec6[k]:=x[k]; END_FOR;
                OK:=TRUE; fkDone:=TRUE;
                EXIT;
            END_IF;
            IF maxerr > 1.0E9 THEN
                EXIT;
            END_IF;

            (* 列归一化（fk 不做 cn 零保护，与 Python 一致） *)
            FOR c2:=1 TO 6 DO
                ssum:=0.0;
                FOR rr:=1 TO 6 DO ssum:=ssum+J[rr,c2]*J[rr,c2]; END_FOR;
                cn[1,c2]:=SQRT(ssum);
            END_FOR;
            FOR rr:=1 TO 6 DO
                FOR c2:=1 TO 6 DO Jn[rr,c2]:=J[rr,c2]/cn[1,c2]; END_FOR;
            END_FOR;
            sol(A:=Jn, b:=fv);
            IF NOT sol.OK THEN EXIT; END_IF;
            FOR c2:=1 TO 6 DO dx6[c2]:=sol.X[c2]/cn[1,c2]; END_FOR;

            (* 步长限幅：位置 50mm / 姿态 0.5rad *)
            step:=ABS(dx6[1]);
            IF ABS(dx6[2])>step THEN step:=ABS(dx6[2]); END_IF;
            IF ABS(dx6[3])>step THEN step:=ABS(dx6[3]); END_IF;
            step:=step/50.0;
            FOR k:=4 TO 6 DO
                tmp:=ABS(dx6[k])/0.5;
                IF tmp>step THEN step:=tmp; END_IF;
            END_FOR;
            IF step>1.0 THEN
                FOR k:=1 TO 6 DO dx6[k]:=dx6[k]/step; END_FOR;
            END_IF;
            FOR k:=1 TO 6 DO x[k]:=x[k]-dx6[k]; END_FOR;
        END_FOR;
        (* fkDone=FALSE 时 OK 保持 FALSE（迭代耗尽/发散/奇异） *)

    ELSE
        (* ---------------- LM fk_track() ---------------- *)
        lam:=1.0E-3;
        fkDone:=FALSE;
        FOR it:=1 TO 40 DO
            px:=x[1]; py:=x[2]; pz:=x[3]; a4:=x[4]; b4:=x[5]; c4:=x[6];
            ca:=COS(a4); sa:=SIN(a4); cb:=COS(b4); sb:=SIN(b4);
            cc:=COS(c4); sc:=SIN(c4);
            maxerr:=0.0;
            FOR k:=1 TO 6 DO
                ki:=((k-1) MOD 3)+1;
                IF k<=3 THEN radP:=r; ELSE radP:=ra; END_IF;
                IF k<=3 THEN rodV:=l; ELSE rodV:=lb; END_IF;
                lax:=radP*CPH[ki]; lay:=radP*SPH[ki]; laz:=0.0;
                a1x:=lax; a1y:=ca*lay-sa*laz; a1z:=sa*lay+ca*laz;
                a2x:=cb*a1x+sb*a1z; a2y:=a1y; a2z:=-sb*a1x+cb*a1z;
                rax:=cc*a2x-sc*a2y; ray:=sc*a2x+cc*a2y; raz:=a2z;
                t1x:=sb*a1y; t1y:=-a1z; t1z:=cb*a1y;
                dax:=cc*t1x-sc*t1y; day:=sc*t1x+cc*t1y; daz:=t1z;
                dbx:=cc*a2z; dby:=sc*a2z; dbz:=-a2x;
                dcx:=-ray; dcy:=rax; dcz:=0.0;
                ax:=px+rax; ay:=py+ray; az:=pz+raz;
                exv:=Eb[k,1]; eyv:=Eb[k,2]; ezv:=Eb[k,3];
                dxv:=ax-exv; dyv:=ay-eyv; dzv:=az-ezv;
                dist:=SQRT(dxv*dxv+dyv*dyv+dzv*dzv);
                err:=ABS(dist-rodV);
                IF err>maxerr THEN maxerr:=err; END_IF;
                fv[k]:=dxv*dxv+dyv*dyv+dzv*dzv-rodV*rodV;
                J[k,1]:=2.0*dxv; J[k,2]:=2.0*dyv; J[k,3]:=2.0*dzv;
                J[k,4]:=2.0*(dxv*dax+dyv*day+dzv*daz);
                J[k,5]:=2.0*(dxv*dbx+dyv*dby+dzv*dbz);
                J[k,6]:=2.0*(dxv*dcx+dyv*dcy+dzv*dcz);
            END_FOR;

            IF maxerr < 1.0E-8 THEN
                FOR k:=1 TO 6 DO OutVec6[k]:=x[k]; END_FOR;
                OK:=TRUE; fkDone:=TRUE;
                EXIT;
            END_IF;

            (* 列归一化（零列保护 1e-14） *)
            FOR c2:=1 TO 6 DO
                ssum:=0.0;
                FOR rr:=1 TO 6 DO ssum:=ssum+J[rr,c2]*J[rr,c2]; END_FOR;
                ssum:=SQRT(ssum);
                IF ssum>1.0E-14 THEN cn[1,c2]:=ssum; ELSE cn[1,c2]:=1.0; END_IF;
            END_FOR;
            FOR rr:=1 TO 6 DO
                FOR c2:=1 TO 6 DO Jn[rr,c2]:=J[rr,c2]/cn[1,c2]; END_FOR;
            END_FOR;
            FOR rr:=1 TO 6 DO
                FOR c2:=1 TO 6 DO
                    ssum:=0.0; nsc:=0.0;
                    FOR k:=1 TO 6 DO
                        ntv:=Jn[k,rr]*Jn[k,c2];
                        ntt:=ssum+ntv;
                        IF ABS(ssum) >= ABS(ntv) THEN
                            nsc:=nsc+((ssum-ntt)+ntv);
                        ELSE
                            nsc:=nsc+((ntv-ntt)+ssum);
                        END_IF;
                        ssum:=ntt;
                    END_FOR;
                    JtJ[rr,c2]:=ssum+nsc;
                END_FOR;
            END_FOR;
            FOR rr:=1 TO 6 DO
                ssum:=0.0; nsc:=0.0;
                FOR k:=1 TO 6 DO
                    ntv:=Jn[k,rr]*fv[k];
                    ntt:=ssum+ntv;
                    IF ABS(ssum) >= ABS(ntv) THEN
                        nsc:=nsc+((ssum-ntt)+ntv);
                    ELSE
                        nsc:=nsc+((ntv-ntt)+ssum);
                    END_IF;
                    ssum:=ntt;
                END_FOR;
                Jtf[rr]:=ssum+nsc;
            END_FOR;
            err0:=0.0; nsc:=0.0;
            FOR k:=1 TO 6 DO
                ntv:=fv[k]*fv[k];
                ntt:=err0+ntv;
                IF ABS(err0) >= ABS(ntv) THEN
                    nsc:=nsc+((err0-ntt)+ntv);
                ELSE
                    nsc:=nsc+((ntv-ntt)+err0);
                END_IF;
                err0:=ntt;
            END_FOR;
            err0:=err0+nsc;

            accepted:=FALSE;
            FOR tries:=1 TO 12 DO
                FOR rr:=1 TO 6 DO
                    FOR c2:=1 TO 6 DO
                        Mm[rr,c2]:=JtJ[rr,c2];
                        IF rr=c2 THEN Mm[rr,c2]:=Mm[rr,c2]+lam; END_IF;
                    END_FOR;
                END_FOR;
                sol(A:=Mm, b:=Jtf);
                IF sol.OK THEN
                    FOR c2:=1 TO 6 DO xn[c2]:=x[c2]-sol.X[c2]/cn[1,c2]; END_FOR;
                    (* 试探步残差 f1（只需残差，数值与 Python 丢弃 J/mx 等价） *)
                    ca:=COS(xn[4]); sa:=SIN(xn[4]); cb:=COS(xn[5]); sb:=SIN(xn[5]);
                    cc:=COS(xn[6]); sc:=SIN(xn[6]);
                    e1:=0.0; nsc:=0.0;
                    FOR k:=1 TO 6 DO
                        ki:=((k-1) MOD 3)+1;
                        IF k<=3 THEN radP:=r; ELSE radP:=ra; END_IF;
                        IF k<=3 THEN rodV:=l; ELSE rodV:=lb; END_IF;
                        lax:=radP*CPH[ki]; lay:=radP*SPH[ki]; laz:=0.0;
                        a1x:=lax; a1y:=ca*lay-sa*laz; a1z:=sa*lay+ca*laz;
                        a2x:=cb*a1x+sb*a1z; a2y:=a1y; a2z:=-sb*a1x+cb*a1z;
                        rax:=cc*a2x-sc*a2y; ray:=sc*a2x+cc*a2y; raz:=a2z;
                        dxv:=xn[1]+rax-Eb[k,1];
                        dyv:=xn[2]+ray-Eb[k,2];
                        dzv:=xn[3]+raz-Eb[k,3];
                        f1[k]:=dxv*dxv+dyv*dyv+dzv*dzv-rodV*rodV;
                        ntv:=f1[k]*f1[k];
                        ntt:=e1+ntv;
                        IF ABS(e1) >= ABS(ntv) THEN
                            nsc:=nsc+((e1-ntt)+ntv);
                        ELSE
                            nsc:=nsc+((ntv-ntt)+e1);
                        END_IF;
                        e1:=ntt;
                    END_FOR;
                    e1:=e1+nsc;
                    IF e1 < err0 THEN
                        FOR c2:=1 TO 6 DO x[c2]:=xn[c2]; END_FOR;
                        lam:=lam/3.0;
                        IF lam<1.0E-9 THEN lam:=1.0E-9; END_IF;
                        accepted:=TRUE;
                        EXIT;
                    END_IF;
                    lam:=lam*4.0;
                ELSE
                    lam:=lam*5.0;
                END_IF;
            END_FOR;

            IF NOT accepted THEN EXIT; END_IF;
            IF maxerr > 1.0E9 THEN EXIT; END_IF;
        END_FOR;
    END_IF;

ELSIF Mode=3 THEN

    (* =======================================================================
       IK 六链闭式解（对应 ik()）。任一链无解：OK=FALSE 立即返回。
       ======================================================================= *)
    px:=Pose6[1]; py:=Pose6[2]; pz:=Pose6[3];
    a4:=Pose6[4]; b4:=Pose6[5]; c4:=Pose6[6];
    ca:=COS(a4); sa:=SIN(a4); cb:=COS(b4); sb:=SIN(b4);
    cc:=COS(c4); sc:=SIN(c4);
    FOR k:=1 TO 6 DO OutVec6[k]:=0.0; END_FOR;

    ikAll:=TRUE;
    FOR k:=1 TO 3 DO
        j3:=k+3;
        cph:=CPH[k]; sph:=SPH[k];

        (* --- 主链 --- *)
        lax:=r*cph; lay:=r*sph; laz:=0.0;
        a1x:=lax; a1y:=ca*lay-sa*laz; a1z:=sa*lay+ca*laz;
        a2x:=cb*a1x+sb*a1z; a2y:=a1y; a2z:=-sb*a1x+cb*a1z;
        rax:=cc*a2x-sc*a2y; ray:=sc*a2x+cc*a2y; raz:=a2z;
        ax:=px+rax; ay:=py+ray; az:=pz+raz;
        u:=ax*cph+ay*sph-R;
        v:=-ax*sph+ay*cph;
        w:=az;
        armV:=L; rodV:=l; thp:=Ref6[k];
        (* solve_chain 内联 *)
        found:=FALSE; thOut:=0.0; bestDD:=0.0;
        m2:=u*u+w*w;
        IF m2>=1.0E-12 THEN
            mV:=SQRT(m2);
            cv:=(armV*armV+m2+v*v-rodV*rodV)/(2.0*armV);
            xx:=cv/mV;
            IF xx<=1.0 AND xx>=-1.0 THEN
                alpha:=ATAN2(w,u); dd:=ACOS(xx);
                (* 支1：alpha+dd *)
                tw:=alpha+dd;
                tw:=tw-PI2*FLOOR(tw/PI2);
                IF SIN(tw)>1.0E-9 THEN
                    dad:=thp-PI2*FLOOR(thp/PI2);
                    dad:=ABS(tw-dad);
                    dad:=dad-PI2*FLOOR(dad/PI2);
                    IF dad>3.141592653589793 THEN dad:=PI2-dad; END_IF;
                    bestDD:=dad; thOut:=tw; found:=TRUE;
                END_IF;
                (* 支2：alpha-dd *)
                tw:=alpha-dd;
                tw:=tw-PI2*FLOOR(tw/PI2);
                IF SIN(tw)>1.0E-9 THEN
                    dad:=thp-PI2*FLOOR(thp/PI2);
                    dad:=ABS(tw-dad);
                    dad:=dad-PI2*FLOOR(dad/PI2);
                    IF dad>3.141592653589793 THEN dad:=PI2-dad; END_IF;
                    IF (NOT found) OR (dad<bestDD) THEN
                        bestDD:=dad; thOut:=tw; found:=TRUE;
                    END_IF;
                END_IF;
            END_IF;
        END_IF;
        IF NOT found THEN OK:=FALSE; ikAll:=FALSE; EXIT; END_IF;
        OutVec6[k]:=thOut;

        (* --- 副链 --- *)
        lax:=ra*cph; lay:=ra*sph; laz:=0.0;
        a1x:=lax; a1y:=ca*lay-sa*laz; a1z:=sa*lay+ca*laz;
        a2x:=cb*a1x+sb*a1z; a2y:=a1y; a2z:=-sb*a1x+cb*a1z;
        rax:=cc*a2x-sc*a2y; ray:=sc*a2x+cc*a2y; raz:=a2z;
        ax:=px+rax; ay:=py+ray; az:=pz+raz;
        u:=ax*cph+ay*sph-Rac;
        v:=-ax*sph+ay*cph;
        w:=az;
        armV:=La; rodV:=lb; thp:=Ref6[j3];
        found:=FALSE; thOut:=0.0; bestDD:=0.0;
        m2:=u*u+w*w;
        IF m2>=1.0E-12 THEN
            mV:=SQRT(m2);
            cv:=(armV*armV+m2+v*v-rodV*rodV)/(2.0*armV);
            xx:=cv/mV;
            IF xx<=1.0 AND xx>=-1.0 THEN
                alpha:=ATAN2(w,u); dd:=ACOS(xx);
                tw:=alpha+dd;
                tw:=tw-PI2*FLOOR(tw/PI2);
                IF SIN(tw)>1.0E-9 THEN
                    dad:=thp-PI2*FLOOR(thp/PI2);
                    dad:=ABS(tw-dad);
                    dad:=dad-PI2*FLOOR(dad/PI2);
                    IF dad>3.141592653589793 THEN dad:=PI2-dad; END_IF;
                    bestDD:=dad; thOut:=tw; found:=TRUE;
                END_IF;
                tw:=alpha-dd;
                tw:=tw-PI2*FLOOR(tw/PI2);
                IF SIN(tw)>1.0E-9 THEN
                    dad:=thp-PI2*FLOOR(thp/PI2);
                    dad:=ABS(tw-dad);
                    dad:=dad-PI2*FLOOR(dad/PI2);
                    IF dad>3.141592653589793 THEN dad:=PI2-dad; END_IF;
                    IF (NOT found) OR (dad<bestDD) THEN
                        bestDD:=dad; thOut:=tw; found:=TRUE;
                    END_IF;
                END_IF;
            END_IF;
        END_IF;
        IF NOT found THEN OK:=FALSE; ikAll:=FALSE; EXIT; END_IF;
        OutVec6[j3]:=thOut;
    END_FOR;
    IF ikAll THEN OK:=TRUE; END_IF;

ELSIF Mode=4 THEN

    (* =======================================================================
       ik_vel 解析关节速度（对应 ik_vel()）
       ======================================================================= *)
    px:=Pose6[1]; py:=Pose6[2]; pz:=Pose6[3];
    a4:=Pose6[4]; b4:=Pose6[5]; c4:=Pose6[6];
    vx:=Twist6[1]; vy:=Twist6[2]; vz:=Twist6[3];
    wa:=Twist6[4]; wb:=Twist6[5]; wc:=Twist6[6];
    ca:=COS(a4); sa:=SIN(a4); cb:=COS(b4); sb:=SIN(b4);
    cc:=COS(c4); sc:=SIN(c4);

    FOR k:=1 TO 3 DO
        j3:=k+3;
        cph:=CPH[k]; sph:=SPH[k];

        (* 主链 *)
        thp:=Ang6[k];
        ce:=COS(thp); se:=SIN(thp);
        lax:=r*cph; lay:=r*sph; laz:=0.0;
        a1x:=lax; a1y:=ca*lay-sa*laz; a1z:=sa*lay+ca*laz;
        a2x:=cb*a1x+sb*a1z; a2y:=a1y; a2z:=-sb*a1x+cb*a1z;
        rax:=cc*a2x-sc*a2y; ray:=sc*a2x+cc*a2y; raz:=a2z;
        t1x:=sb*a1y; t1y:=-a1z; t1z:=cb*a1y;
        dax:=cc*t1x-sc*t1y; day:=sc*t1x+cc*t1y; daz:=t1z;
        dbx:=cc*a2z; dby:=sc*a2z; dbz:=-a2x;
        dcx:=-ray; dcy:=rax; dcz:=0.0;
        dxv:=px+rax-(R+L*ce)*cph;
        dyv:=py+ray-(R+L*ce)*sph;
        dzv:=pz+raz-L*se;
        wx:=dax*wa+dbx*wb+dcx*wc;
        wy:=day*wa+dby*wb+dcy*wc;
        wz:=daz*wa+dbz*wb+dcz*wc;
        num:=dxv*(vx+wx)+dyv*(vy+wy)+dzv*(vz+wz);
        den:=dxv*(-L*se*cph)+dyv*(-L*se*sph)+dzv*(L*ce);
        OutVec6[k]:=num/den;

        (* 副链 *)
        thp:=Ang6[j3];
        ce:=COS(thp); se:=SIN(thp);
        lax:=ra*cph; lay:=ra*sph; laz:=0.0;
        a1x:=lax; a1y:=ca*lay-sa*laz; a1z:=sa*lay+ca*laz;
        a2x:=cb*a1x+sb*a1z; a2y:=a1y; a2z:=-sb*a1x+cb*a1z;
        rax:=cc*a2x-sc*a2y; ray:=sc*a2x+cc*a2y; raz:=a2z;
        t1x:=sb*a1y; t1y:=-a1z; t1z:=cb*a1y;
        dax:=cc*t1x-sc*t1y; day:=sc*t1x+cc*t1y; daz:=t1z;
        dbx:=cc*a2z; dby:=sc*a2z; dbz:=-a2x;
        dcx:=-ray; dcy:=rax; dcz:=0.0;
        dxv:=px+rax-(Rac+La*ce)*cph;
        dyv:=py+ray-(Rac+La*ce)*sph;
        dzv:=pz+raz-La*se;
        wx:=dax*wa+dbx*wb+dcx*wc;
        wy:=day*wa+dby*wb+dcy*wc;
        wz:=daz*wa+dbz*wb+dcz*wc;
        num:=dxv*(vx+wx)+dyv*(vy+wy)+dzv*(vz+wz);
        den:=dxv*(-La*se*cph)+dyv*(-La*se*sph)+dzv*(La*ce);
        OutVec6[j3]:=num/den;
    END_FOR;
    OK:=TRUE;

END_IF;

END_FUNCTION_BLOCK
