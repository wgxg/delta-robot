# -*- coding: utf-8 -*-
"""
test_st6.py —— P2 / M3 验收：ST 移植件 vs Python 黄金参考【逐拍对拍】
=====================================================================
两个环境并行驱动【完全相同】的指令序列（同周期、同 Execute 时序、同参数、
同伺服一阶跟随 gain=0.3），每拍在核心执行后、电机更新前逐字段比较：
  · 轴 BOOL 信号严格相等（Busy/InPos/hasBuf/FkError/Enable/...）
  · owner/cmdCount/ActiveType 严格相等
  · TarTheta[1..6] 逐角 |Δ| < 1e-3 rad                （M3 主判据）
  · Plan/Cur 位置 < 1e-3 mm、姿态 < 1e-3 度、速度 < 1e-3
  · 每个指令 FB 的 Busy/InPos/CommandAborted/Rejected 严格相等
ST 侧执行的是 st_runtime.py 从 6 个真实 .st 文件加载的解释结果
（无转写、无数值桩）。
"""

import os
import sys
import math

import st_runtime as st
import test_delta6 as G
from kinematics6 import ik, pose_rad, PARAMS

HERE = os.path.dirname(os.path.abspath(__file__))
ST_FILES = ['DeltaTypes6.st', 'FB_DeltaKinematics6.st', 'FB_DeltaCore6.st',
            'FB_PTP6.st', 'FB_LIN6.st', 'FB_CIR6.st']

TOL = 1.0E-3
BOOL_AXIS = ('FfEnable', 'Enable', 'FkError', 'Busy', 'InPos', 'hasBuf')
INT_AXIS = ('ActiveType', 'owner', 'cmdCount')
LIST_AXIS = ('CurTheta', 'TarTheta')
FB_SIGNALS = ('Busy', 'InPos', 'CommandAborted', 'Rejected')


def build_world():
    w = st.World()
    for fn in ST_FILES:
        with open(os.path.join(HERE, fn), 'r', encoding='utf-8') as f:
            w.load_text(f.read())
    return w


class Mismatch(Exception):
    pass


class Pair:
    """一组 Python/ST 双环境（轴+核心+若干指令FB+伺服角）。"""

    def __init__(self, world):
        self.w = world
        self.py_axis = G.DeltaAxis6()
        self.st_axis = world.new_struct('st_DeltaAxis6')
        self.py_core = G.FB_DeltaCore6()
        self.st_core = world.new_fb('FB_DeltaCore6')
        home = ik(pose_rad(G.HOME_DEG))
        self.py_th = list(home)
        self.st_th = list(home)
        self.fbs = {}          # key -> (pyfb, stfb, type)
        self.cycle = 0

    def add_fb(self, key, fb_type):
        pyfb = getattr(G, fb_type)()
        stfb = self.w.new_fb(fb_type)
        self.fbs[key] = (pyfb, stfb, fb_type)
        return pyfb, stfb

    def _core_kwargs(self, ff_enable, kff, enable):
        return dict(Enable=enable,
                    FfEnable=ff_enable if enable else False,
                    Kff=kff if enable else 0.0)

    def step(self, calls, ff_enable=True, kff=0.5, enable=True,
             check=True):
        """calls: [(fbkey, Execute(bool), params-dict), ...]"""
        # 1) 写反馈角
        for k in range(6):
            self.py_axis.CurTheta[k] = self.py_th[k]
            self.st_axis['CurTheta'][k + 1] = self.st_th[k]

        # 2) 指令 FB（两侧同序）
        for key, ex, p in calls:
            pyfb, stfb, _t = self.fbs[key]
            pyfb(self.py_axis, Execute=ex, **p)
            self.w.call(stfb, Execute=ex, Axis=self.st_axis, **p)

        # 3) 核心
        ck = self._core_kwargs(ff_enable, kff, enable)
        self.py_core(self.py_axis, **ck)
        self.w.call(self.st_core,
                    R=PARAMS['R'], r=PARAMS['r'], L=PARAMS['L'], l=PARAMS['l'],
                    Rac=PARAMS['Rac'], ra=PARAMS['ra'], La=PARAMS['La'],
                    lb=PARAMS['lb'], CycleTime=G.CYCLE,
                    FfEnable=ck['FfEnable'], Kff=ck['Kff'],
                    Enable=ck['Enable'], Axis=self.st_axis)

        # 4) 逐拍比较
        if check:
            self.compare()

        # 5) 一阶伺服跟随
        for k in range(6):
            self.py_th[k] += (self.py_axis.TarTheta[k] - self.py_th[k]) * 0.3
            self.st_th[k] += (self.st_axis['TarTheta'][k + 1] - self.st_th[k]) * 0.3
        self.cycle += 1

    def compare(self):
        pa, sa = self.py_axis, self.st_axis
        where = 'cycle=%d' % self.cycle
        for field in sa:
            pyv = getattr(pa, field)
            stv = sa[field]
            if field in BOOL_AXIS:
                if bool(pyv) != bool(stv):
                    raise Mismatch('%s %s: py=%r st=%r'
                                   % (where, field, pyv, stv))
            elif field in INT_AXIS:
                if int(pyv) != int(stv):
                    raise Mismatch('%s %s: py=%r st=%r'
                                   % (where, field, pyv, stv))
            elif field in LIST_AXIS:
                for k in range(6):
                    d = abs(float(pyv[k]) - float(stv[k + 1]))
                    if d > TOL:
                        raise Mismatch('%s %s[%d]: py=%.9f st=%.9f Δ=%.3e'
                                       % (where, field, k + 1, pyv[k],
                                          stv[k + 1], d))
            else:
                d = abs(float(pyv) - float(stv))
                if d > TOL:
                    raise Mismatch('%s %s: py=%.9f st=%.9f Δ=%.3e'
                                   % (where, field, pyv, stv, d))
        for key, (pyfb, stfb, _t) in self.fbs.items():
            for sig in FB_SIGNALS:
                pv = getattr(pyfb, sig)
                sv = stfb.vars[sig]
                if bool(pv) != bool(sv):
                    raise Mismatch('%s fb[%s].%s: py=%r st=%r'
                                   % (where, key, sig, pv, sv))

    def run_done(self, key, p, exec_cycles=1, max_cycles=8000, settle=20,
                 **ckw):
        """驱动单指令到完成（以 Python InPos 为准），含 settle 拍。"""
        pyfb = self.fbs[key][0]
        done_cycle = -1
        for i in range(max_cycles):
            self.step([(key, i < exec_cycles, p)], **ckw)
            if i >= exec_cycles and pyfb.InPos and not pyfb.Busy:
                done_cycle = i
                break
        if done_cycle < 0:
            raise Mismatch('%s: 超时未完成' % key)
        for _ in range(settle):
            self.step([(key, False, p)], **ckw)
        return done_cycle


# ============================================================
# 场景
# ============================================================
def setup(world):
    pr = Pair(world)
    pr.step([])                       # 先空跑一拍使能核心
    return pr


results = {'pass': 0, 'fail': 0}


def t_case(name, fn, world):
    try:
        fn(world)
        results['pass'] += 1
        print('  [PASS] %s' % name)
    except Exception as e:
        results['fail'] += 1
        print('  [FAIL] %s  %s' % (name, e))


def s1(world):
    pr = setup(world); pr.add_fb('p', 'FB_PTP6')
    c = pr.run_done('p', G.ptp6(30, 0, 300, speed=200, accel=1000))
    assert c > 0


def s2(world):
    pr = setup(world); pr.add_fb('p', 'FB_PTP6')
    c = pr.run_done('p', G.ptp6(-20, 25, 290, a=8, b=-5, c=10, speed=150,
                                accel=800, jerk=8000, move_mode=1))
    assert c > 0


def s3(world):
    pr = setup(world)
    pr.add_fb('h', 'FB_PTP6'); pr.add_fb('l', 'FB_LIN6')
    pr.run_done('h', G.ptp6(0, 0, 300))
    c = pr.run_done('l', G.lin6(-30, -20, 285, a=-6, b=8, c=-12, speed=180,
                                accel=900, jerk=9000, move_mode=1))
    assert c > 0


def s4(world):
    pr = setup(world)
    pr.add_fb('h', 'FB_PTP6'); pr.add_fb('c', 'FB_CIR6')
    pr.run_done('h', G.ptp6(30, 0, 300))
    c = pr.run_done('c', G.cir6(10, 20, 300, aux=(10, 0, 300), cir_mode=0))
    assert c > 0


def s5(world):
    pr = setup(world)
    pr.add_fb('h', 'FB_PTP6'); pr.add_fb('c', 'FB_CIR6')
    pr.run_done('h', G.ptp6(30, 0, 300))
    c = pr.run_done('c', G.cir6(-10, 0, 300, a=0, b=0, c=12,
                                aux=(10, 20, 300), cir_mode=1))
    assert c > 0


def s6(world):
    """打断 BufferMode=1"""
    pr = setup(world)
    pr.add_fb('h', 'FB_PTP6'); pr.add_fb('f1', 'FB_LIN6')
    pr.add_fb('f2', 'FB_PTP6')
    pr.run_done('h', G.ptp6(0, 0, 300))
    p1 = G.lin6(35, 0, 300, speed=25, accel=120, buffer_mode=1)
    p2 = G.ptp6(0, 30, 315, b=6, speed=200, accel=1000, buffer_mode=1)
    done = False
    for i in range(2500):
        calls = [('f1', i == 0, p1), ('f2', i == 150, p2)]
        pr.step(calls)
        if i > 150 and pr.fbs['f2'][0].InPos and not pr.fbs['f2'][0].Busy:
            done = True
            break
    assert done
    assert pr.fbs['f1'][0].CommandAborted
    for _ in range(20):
        pr.step([('f1', False, p1), ('f2', False, p2)])


def s7(world):
    """缓存 BufferMode=2 两条连跑"""
    pr = setup(world)
    pr.add_fb('h', 'FB_PTP6'); pr.add_fb('f1', 'FB_LIN6')
    pr.add_fb('f2', 'FB_LIN6')
    pr.run_done('h', G.ptp6(0, 0, 300))
    p1 = G.lin6(35, 0, 300, speed=60, accel=300, buffer_mode=2)
    p2 = G.lin6(35, 25, 300, speed=60, accel=300, buffer_mode=2)
    done = False
    for i in range(6000):
        calls = [('f1', i == 0, p1), ('f2', i == 5, p2)]
        pr.step(calls)
        if pr.fbs['f2'][0].InPos and not pr.fbs['f2'][0].Busy \
                and abs(pr.py_axis.PlanY - 25) < 2:
            done = True
            break
    assert done
    for _ in range(20):
        pr.step([('f1', False, p1), ('f2', False, p2)])


def s8(world):
    """IK 预检拒绝 + Rejected 单拍脉冲 + 拒绝后正常接单"""
    pr = setup(world)
    pr.add_fb('h', 'FB_PTP6'); pr.add_fb('bad', 'FB_PTP6')
    pr.add_fb('ok', 'FB_LIN6')
    pr.run_done('h', G.ptp6(0, 0, 300))
    p_bad = G.ptp6(300, 0, 300, buffer_mode=0)
    pr.step([('bad', True, p_bad)])
    assert pr.fbs['bad'][0].Rejected and not pr.fbs['bad'][0].Busy
    pr.step([('bad', False, p_bad)])
    assert not pr.fbs['bad'][0].Rejected
    pr.run_done('ok', G.lin6(20, 0, 300, buffer_mode=0))


def s9(world):
    """失能/恢复"""
    pr = setup(world)
    pr.add_fb('h', 'FB_PTP6'); pr.add_fb('v', 'FB_LIN6')
    pr.add_fb('f2', 'FB_PTP6')
    pr.run_done('h', G.ptp6(0, 0, 300))
    p = G.lin6(35, 0, 300, speed=25, accel=120)
    for i in range(150):
        pr.step([('v', i == 0, p)])
    assert pr.fbs['v'][0].Busy and pr.py_axis.Busy
    for _ in range(5):
        pr.step([('v', False, p)], enable=False)
    assert not pr.fbs['v'][0].Busy and not pr.py_axis.Busy
    assert all(pr.st_axis['TarTheta'][k] == 0 for k in range(1, 7))
    # 反馈回零位后重新使能
    pr.py_th = list(ik(pose_rad(G.HOME_DEG)))
    pr.st_th = list(ik(pose_rad(G.HOME_DEG)))
    for _ in range(3):
        pr.step([])
    c = pr.run_done('f2', G.ptp6(20, -15, 310, buffer_mode=0))
    assert c > 0
    assert pr.fbs['v'][0].phase == 0


def s10(world):
    """前馈使能/禁用两路各 60 拍（两路都各自逐拍一致）"""
    p = G.ptp6(30, 0, 300, speed=300, accel=2000)
    for ff, kff in ((False, 0.0), (True, 0.5)):
        pr = setup(world); pr.add_fb('p', 'FB_PTP6')
        for i in range(60):
            pr.step([('p', i == 0, p)], ff_enable=ff, kff=kff)


def s11(world):
    """缓存槽满 + 打断清空缓存（测试 10/11 合并）"""
    pr = setup(world)
    pr.add_fb('h', 'FB_PTP6')
    pr.add_fb('f1', 'FB_LIN6'); pr.add_fb('f2', 'FB_LIN6')
    pr.add_fb('f3', 'FB_PTP6')
    pr.run_done('h', G.ptp6(0, 0, 300))
    p1 = G.lin6(35, 0, 300, speed=25, accel=120, buffer_mode=2)
    p2 = G.lin6(0, 30, 300, speed=25, accel=120, buffer_mode=2)
    p3 = G.ptp6(0, 0, 315, speed=200, accel=1000, buffer_mode=1)
    for i in range(11):
        calls = [('f1', i == 0, p1), ('f2', i == 5, p2), ('f3', False, p3)]
        if i == 10:
            calls[2] = ('f3', True, p3)
        pr.step(calls)
    assert not pr.fbs['f2'][0].Rejected          # 第 5 拍缓存成功
    seen = False
    for _ in range(400):
        pr.step([('f1', False, p1), ('f2', False, p2), ('f3', False, p3)])
        if pr.fbs['f2'][0].CommandAborted and not pr.py_axis.hasBuf:
            seen = True
            break
    assert seen
    assert pr.fbs['f3'][0].Busy and pr.st_axis['ActiveType'] == 1


def s12(world):
    """大行程 PTP S 曲线 + 纯姿态 LIN（测试 15/16 合并）"""
    pr = setup(world)
    pr.add_fb('h', 'FB_PTP6'); pr.add_fb('p', 'FB_PTP6')
    pr.run_done('h', G.ptp6(0, 0, 320))
    c = pr.run_done('p', G.ptp6(-30, 25, 285, a=-8, b=8, c=-15, speed=200,
                                accel=1500, move_mode=1), max_cycles=8000)
    assert c > 0

    pr2 = setup(world); pr2.add_fb('l', 'FB_LIN6')
    c2 = pr2.run_done('l', G.lin6(0, 0, 300, a=10, b=-8, c=15, speed=60,
                                  accel=300))
    assert c2 > 0


def s13(world):
    """连续 3 段 LIN（含姿态）无跳变"""
    pr = setup(world)
    pr.add_fb('h', 'FB_PTP6'); pr.add_fb('l', 'FB_LIN6')
    pr.run_done('h', G.ptp6(0, 0, 300))
    pts = [(25, 0, 300, 0, 0, 0), (25, 25, 305, 5, 5, 0),
           (0, 25, 300, 0, 0, 8)]
    for pt in pts:
        c = pr.run_done('l', G.lin6(*pt, speed=120, accel=700))
        assert c > 0


SCENES = [
    ('场景1 PTP 梯形曲线', s1),
    ('场景2 PTP S曲线(带姿态)', s2),
    ('场景3 LIN S曲线(slerp 姿态)', s3),
    ('场景4 CIR 圆心式', s4),
    ('场景5 CIR 三点式(带姿态)', s5),
    ('场景6 打断 BufferMode=1', s6),
    ('场景7 缓存 BufferMode=2 连跑', s7),
    ('场景8 IK 预检拒绝+脉冲+后续接单', s8),
    ('场景9 失能/恢复', s9),
    ('场景10 前馈使能/禁用', s10),
    ('场景11 缓存槽满+打断清空缓存', s11),
    ('场景12 大行程S曲线+纯姿态LIN', s12),
    ('场景13 连续3段LIN', s13),
]


if __name__ == '__main__':
    print('加载真实 .st 文件构建 ST 运行时 ...')
    world = build_world()
    print('  类型: %s' % sorted(world.types))
    print('  功能块: %s' % sorted(world.fbdefs))
    print('=' * 64)
    print('ST6 逐拍对拍（阈值 Tar/Plan/Cur |Δ| < 1e-3，信号严格相等）')
    print('=' * 64)
    for name, fn in SCENES:
        print('-' * 64)
        print(name)
        t_case(name, fn, world)
    print('=' * 64)
    print('对拍结果: %d 通过, %d 失败' % (results['pass'], results['fail']))
    print('=' * 64)
    sys.exit(1 if results['fail'] else 0)
