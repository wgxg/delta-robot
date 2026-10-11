# -*- coding: utf-8 -*-
"""
st_runtime.py —— P2 交付：ST(IEC 61131-3) 子集解释器
=====================================================================
直接加载并执行本目录 6 个真实 .st 源文件（DeltaTypes6.st /
FB_DeltaKinematics6.st / FB_DeltaCore6.st / FB_PTP6.st / FB_LIN6.st /
FB_CIR6.st），供 test_st6.py 与 Python 黄金参考 test_delta6.py 逐拍对拍。

支持的 ST 子集（即 6 个文件实际使用的语法）：
  · TYPE/STRUCT/END_STRUCT/END_TYPE（BOOL/INT/DINT/LREAL、一维/二维数组）
  · FUNCTION_BLOCK：VAR_INPUT / VAR_IN_OUT / VAR_OUTPUT / VAR
    （含标量/数组初值、FB 实例成员）
  · IF/ELSIF/ELSE/END_IF、FOR..TO..BY..DO/END_FOR、EXIT、RETURN
  · 赋值、多语句同行、命名实参 FB 调用 inst(Arg:=expr, ...)（可跨行）
  · 表达式：+ - * / MOD、= <> < <= > >=、AND OR NOT、一元负、括号、
    多维下标 a[i,j]、成员访问 inst.Out / Axis.Field
  · 内置：SQRT SIN COS ASIN ACOS ATAN2 ABS FLOOR RAD DEG
全部数值按 Python float / int 运算（ST 侧 LREAL 双精度）。
"""

import math
import copy


# ============================================================
# 词法
# ============================================================
def strip_comments(src):
    """剥离 (* ... *) 注释（支持嵌套）。"""
    out = []
    i, depth, n = 0, 0, len(src)
    while i < n:
        if src[i:i + 2] == '(*':
            depth += 1
            i += 2
            continue
        if src[i:i + 2] == '*)':
            depth -= 1
            i += 2
            continue
        if depth == 0:
            out.append(src[i])
        i += 1
    return ''.join(out)


_TOK_SPEC = [
    ('ASSIGN',  r':='),
    ('RANGE',   r'\.\.'),
    ('NEQ',     r'<>'),
    ('LE',      r'<='),
    ('GE',      r'>='),
    ('NUM',     r'\d+\.\d+(?:[eE][+-]?\d+)?|\d+(?:[eE][+-]?\d+)?'),
    ('ID',      r'[A-Za-z_][A-Za-z0-9_]*'),
    ('PUNCT',   r'[+\-*/=<>():,;.\[\]]'),
]
_TOK_RE = __import__('re').compile('|'.join('(?P<%s>%s)' % (k, p)
                                             for k, p in _TOK_SPEC))


def tokenize(src):
    toks = []
    i, n = 0, len(src)
    while i < n:
        ch = src[i]
        if ch.isspace():
            i += 1
            continue
        m = _TOK_RE.match(src, i)
        if not m:
            raise SyntaxError('无法词法分析: %r (pos=%d)' % (src[i:i + 20], i))
        kind = m.lastgroup
        val = m.group()
        if kind == 'NUM':
            val = float(val) if ('.' in val or 'e' in val or 'E' in val) \
                else int(val)
        toks.append((kind, val))
        i = m.end()
    return toks


# ============================================================
# 语法分析
# ============================================================
KW = {'IF', 'THEN', 'ELSIF', 'ELSE', 'END_IF', 'FOR', 'TO', 'BY', 'DO',
      'END_FOR', 'EXIT', 'RETURN', 'FUNCTION_BLOCK', 'END_FUNCTION_BLOCK',
      'VAR_INPUT', 'VAR_IN_OUT', 'VAR_OUTPUT', 'VAR', 'END_VAR',
      'TYPE', 'STRUCT', 'END_STRUCT', 'END_TYPE', 'ARRAY', 'OF',
      'AND', 'OR', 'NOT', 'MOD', 'TRUE', 'FALSE'}

BUILTIN = {'SQRT', 'SIN', 'COS', 'ASIN', 'ACOS', 'ATAN2', 'ABS', 'FLOOR',
           'RAD', 'DEG'}


class Parser:
    def __init__(self, toks, fb_names):
        self.t = toks
        self.p = 0
        self.fb_names = fb_names          # FUNCTION_BLOCK 名集合
        self._fb_inst_names = set()       # 当前 FB 内的实例成员名

    # ---------- 基础工具 ----------
    def peek(self, k=0):
        j = self.p + k
        return self.t[j] if j < len(self.t) else ('EOF', None)

    def next(self):
        tk = self.t[self.p]
        self.p += 1
        return tk

    def accept_id(self, word):
        if self.peek() == ('ID', word):
            self.p += 1
            return True
        return False

    def expect_id(self, word):
        if not self.accept_id(word):
            raise SyntaxError('期望 %s，实际 %r' % (word, self.peek()))

    def accept_punct(self, ch):
        if self.peek() == ('PUNCT', ch):
            self.p += 1
            return True
        return False

    def expect_punct(self, ch):
        if not self.accept_punct(ch):
            raise SyntaxError('期望 %r，实际 %r' % (ch, self.peek()))

    def is_kw(self, word):
        return self.peek() == ('ID', word)

    # ---------- 顶层 ----------
    def parse_unit(self):
        types, fbs = {}, {}
        while self.peek()[0] != 'EOF':
            if self.is_kw('TYPE'):
                name, fields = self.parse_type()
                types[name] = fields
            elif self.is_kw('FUNCTION_BLOCK'):
                fb = self.parse_fb_header_and_body()
                fbs[fb['name']] = fb
            else:
                raise SyntaxError('顶层非法 token %r' % (self.peek(),))
        return types, fbs

    def parse_type(self):
        self.expect_id('TYPE')
        name = self.next()[1]
        self.expect_punct(':')
        self.expect_id('STRUCT')
        fields = []
        while not self.is_kw('END_STRUCT'):
            fields.extend(self.parse_var_decls())
        self.expect_id('END_STRUCT')
        self.expect_id('END_TYPE')
        return name, fields

    def parse_fb_header_and_body(self):
        self.expect_id('FUNCTION_BLOCK')
        name = self.next()[1]
        sections = {'in': [], 'inout': [], 'out': [], 'var': []}
        while True:
            if self.is_kw('VAR_INPUT'):
                self.next()
                while not self.is_kw('END_VAR'):
                    sections['in'].extend(self.parse_var_decls())
                self.expect_id('END_VAR')
            elif self.is_kw('VAR_IN_OUT'):
                self.next()
                while not self.is_kw('END_VAR'):
                    sections['inout'].extend(self.parse_var_decls())
                self.expect_id('END_VAR')
            elif self.is_kw('VAR_OUTPUT'):
                self.next()
                while not self.is_kw('END_VAR'):
                    sections['out'].extend(self.parse_var_decls())
                self.expect_id('END_VAR')
            elif self.is_kw('VAR'):
                self.next()
                while not self.is_kw('END_VAR'):
                    sections['var'].extend(self.parse_var_decls())
                self.expect_id('END_VAR')
            else:
                break
        for sec in sections.values():
            for nm, desc, _i in sec:
                if desc[0] == 'scalar' and desc[1] in self.fb_names:
                    self._fb_inst_names.add(nm)
        body = self.parse_stmt_list({'END_FUNCTION_BLOCK'})
        self.expect_id('END_FUNCTION_BLOCK')
        return {'name': name, 'sections': sections, 'body': body}

    def parse_var_decls(self):
        """一个 名字列表 : 类型 [:= 初值] ; 声明，返回 [(name, desc, init)]"""
        names = [self.next()[1]]
        while self.accept_punct(','):
            names.append(self.next()[1])
        self.expect_punct(':')
        desc = self.parse_type_desc()
        init = None
        if self.peek() == ('ASSIGN', ':='):
            self.next()
            if self.accept_punct('('):
                init = []
                if not self.peek() == ('PUNCT', ')'):
                    init.append(self.const_expr())
                    while self.accept_punct(','):
                        init.append(self.const_expr())
                self.expect_punct(')')
            else:
                init = self.const_expr()
        self.expect_punct(';')
        return [(nm, desc, init) for nm in names]

    def parse_type_desc(self):
        if self.accept_id('ARRAY'):
            self.expect_punct('[')
            bounds = []
            while True:
                lo = int(self.next()[1])
                if self.peek() == ('RANGE', '..'):
                    self.next()
                hi = int(self.next()[1])
                bounds.append((lo, hi))
                if not self.accept_punct(','):
                    break
            self.expect_punct(']')
            self.expect_id('OF')
            base = self.next()[1]
            return ('array', bounds, base)
        base = self.next()[1]
        return ('scalar', base)

    def const_expr(self):
        node = self.parse_expr()
        return ConstEvaluator().eval(node)

    # ---------- 语句 ----------
    def parse_stmt_list(self, stops):
        stmts = []
        while self.peek()[0] != 'EOF' and not (
                self.peek()[0] == 'ID' and self.peek()[1] in stops):
            stmts.append(self.parse_stmt())
        return stmts

    def parse_stmt(self):
        if self.is_kw('IF'):
            return self.parse_if()
        if self.is_kw('FOR'):
            return self.parse_for()
        if self.is_kw('EXIT'):
            self.next(); self.expect_punct(';')
            return ('exit',)
        if self.is_kw('RETURN'):
            self.next(); self.expect_punct(';')
            return ('return',)
        # 赋值 或 FB 调用语句
        target = self.parse_postfix()
        if self.peek() == ('ASSIGN', ':='):
            self.next()
            val = self.parse_expr()
            self.expect_punct(';')
            return ('assign', target, val)
        if target[0] == 'call' and target[3] == 'fb':
            self.expect_punct(';')
            return ('fbcall', target[1], target[2])
        raise SyntaxError('非法语句开头: %r' % (target,))

    def parse_if(self):
        self.expect_id('IF')
        cond = self.parse_expr()
        self.expect_id('THEN')
        branches = [(cond, self.parse_stmt_list({'ELSIF', 'ELSE', 'END_IF'}))]
        els = None
        while self.is_kw('ELSIF'):
            self.next()
            c = self.parse_expr()
            self.expect_id('THEN')
            b = self.parse_stmt_list({'ELSIF', 'ELSE', 'END_IF'})
            branches.append((c, b))
        if self.is_kw('ELSE'):
            self.next()
            els = self.parse_stmt_list({'END_IF'})
        self.expect_id('END_IF')
        self.accept_punct(';')
        return ('if', branches, els)

    def parse_for(self):
        self.expect_id('FOR')
        var = self.next()[1]
        if self.peek() == ('ASSIGN', ':='):
            self.next()
        start = self.parse_expr()
        self.expect_id('TO')
        end = self.parse_expr()
        step = ('num', 1)
        if self.is_kw('BY'):
            self.next()
            step = self.parse_expr()
        self.expect_id('DO')
        body = self.parse_stmt_list({'END_FOR'})
        self.expect_id('END_FOR')
        self.accept_punct(';')
        return ('for', var, start, end, step, body)

    # ---------- 表达式（递归下降） ----------
    def parse_expr(self):
        return self.parse_or()

    def parse_or(self):
        l = self.parse_and()
        while self.is_kw('OR'):
            self.next()
            r = self.parse_and()
            l = ('binop', 'OR', l, r)
        return l

    def parse_and(self):
        l = self.parse_not()
        while self.is_kw('AND'):
            self.next()
            r = self.parse_not()
            l = ('binop', 'AND', l, r)
        return l

    def parse_not(self):
        if self.is_kw('NOT'):
            self.next()
            return ('unop', 'NOT', self.parse_not())
        return self.parse_cmp()

    def parse_cmp(self):
        l = self.parse_add()
        while self.peek()[0] in ('NEQ', 'LE', 'GE') or \
                (self.peek()[0] == 'PUNCT' and self.peek()[1] in '=<>'):
            op = self.next()
            op = {'=': '=', '<>': 'NEQ', '<': '<', '>': '>',
                  '<=': 'LE', '>=': 'GE'}[op[1]]
            r = self.parse_add()
            l = ('binop', op, l, r)
        return l

    def parse_add(self):
        l = self.parse_mul()
        while self.peek() == ('PUNCT', '+') or self.peek() == ('PUNCT', '-'):
            op = self.next()[1]
            r = self.parse_mul()
            l = ('binop', op, l, r)
        return l

    def parse_mul(self):
        l = self.parse_unary()
        while self.peek() == ('PUNCT', '*') or self.peek() == ('PUNCT', '/') \
                or self.is_kw('MOD'):
            if self.is_kw('MOD'):
                self.next(); op = 'MOD'
            else:
                op = self.next()[1]
            r = self.parse_unary()
            l = ('binop', op, l, r)
        return l

    def parse_unary(self):
        if self.peek() == ('PUNCT', '-'):
            self.next()
            return ('unop', '-', self.parse_unary())
        return self.parse_postfix()

    def parse_postfix(self):
        node = self.parse_atom()
        while True:
            if self.accept_punct('.'):
                fld = self.next()[1]
                node = ('field', node, fld)
            elif self.accept_punct('['):
                idxs = [self.parse_expr()]
                while self.accept_punct(','):
                    idxs.append(self.parse_expr())
                self.expect_punct(']')
                node = ('index', node, idxs)
            elif self.accept_punct('('):
                args = []
                if not self.peek() == ('PUNCT', ')'):
                    args.append(self.parse_call_arg())
                    while self.accept_punct(','):
                        args.append(self.parse_call_arg())
                self.expect_punct(')')
                kind = 'fb' if self._is_fb_inst(node) else 'builtin'
                node = ('call', node, args, kind)
            else:
                break
        return node

    def parse_call_arg(self):
        # 命名实参 Name:=expr；否则位置实参
        if self.peek()[0] == 'ID' and self.peek(1) == ('ASSIGN', ':='):
            nm = self.next()[1]
            self.next()
            return (nm, self.parse_expr())
        return (None, self.parse_expr())

    def _is_fb_inst(self, node):
        return node[0] == 'var' and node[1] in self._fb_inst_names

    def parse_atom(self):
        tk = self.peek()
        if tk[0] == 'NUM':
            self.next()
            return ('num', tk[1])
        if tk[0] == 'ID':
            if tk[1] == 'TRUE':
                self.next(); return ('bool', True)
            if tk[1] == 'FALSE':
                self.next(); return ('bool', False)
            self.next()
            return ('var', tk[1])
        if self.accept_punct('('):
            e = self.parse_expr()
            self.expect_punct(')')
            return e
        raise SyntaxError('非法表达式 token %r' % (tk,))


class ConstEvaluator:
    def eval(self, n):
        t = n[0]
        if t == 'num':
            return n[1]
        if t == 'bool':
            return n[1]
        if t == 'unop' and n[1] == '-':
            return -self.eval(n[2])
        if t == 'binop':
            a, b = self.eval(n[2]), self.eval(n[3])
            return {'+': a + b, '-': a - b, '*': a * b, '/': a / b}[n[1]]
        raise SyntaxError('非常量初值 %r' % (n,))


# ============================================================
# 运行时
# ============================================================
class _Return(Exception):
    pass


class _Exit(Exception):
    pass


class FBInstance:
    def __init__(self, defn, world):
        self.type = defn['name']
        self.defn = defn
        self.world = world
        self.vars = {}
        for sec in ('in', 'inout', 'out', 'var'):
            for name, desc, init in defn['sections'][sec]:
                self.vars[name] = world.default_value(desc, init)


class World:
    def __init__(self):
        self.types = {}      # struct name -> fields
        self.fbdefs = {}     # fb name -> defn
        self.fbnames = set()

    def load_text(self, src):
        src = strip_comments(src)
        toks = tokenize(src)
        # 预扫：收集全部 FB 名（本文件 + 此前已加载文件，VAR 实例声明需要）
        names = set(self.fbnames)
        for i, (k, v) in enumerate(toks):
            if k == 'ID' and v == 'FUNCTION_BLOCK' and i + 1 < len(toks):
                names.add(toks[i + 1][1])
        p = Parser(toks, names)
        types, fbs = p.parse_unit()
        self.types.update(types)
        self.fbdefs.update(fbs)
        self.fbnames.update(fbs)

    def default_value(self, desc, init=None):
        kind = desc[0]
        if kind == 'scalar':
            tname = desc[1]
            if tname in self.fbdefs:
                return FBInstance(self.fbdefs[tname], self)
            if init is not None:
                return init
            return False if tname == 'BOOL' else (0 if tname in ('INT', 'DINT')
                                                  else 0.0)
        # array
        bounds = desc[1]

        def build(dim):
            lo, hi = dim[0]
            col = [None] * (hi + 1)
            for i in range(lo, hi + 1):
                col[i] = build(dim[1:]) if len(dim) > 1 else 0.0
            return col

        arr = build(bounds)
        if init:  # 行主序填充聚合量
            flat = []

            def walk(a):
                if isinstance(a, list):
                    for x in a:
                        walk(x)
                else:
                    flat.append(a)

            walk(arr)
            idxs = [b[0] for b in bounds]
            for v in init:
                cur = arr
                for d in range(len(bounds)):
                    if d == len(bounds) - 1:
                        cur[idxs[d]] = v
                    else:
                        cur = cur[idxs[d]]
                # 行主序进位
                d = len(bounds) - 1
                while d >= 0:
                    idxs[d] += 1
                    if idxs[d] <= bounds[d][1]:
                        break
                    idxs[d] = bounds[d][0]
                    d -= 1
        return arr

    def new_struct(self, tname):
        d = {}
        for name, desc, _init in self.types[tname]:
            d[name] = self.default_value(desc)
        return d

    def new_fb(self, tname):
        return FBInstance(self.fbdefs[tname], self)

    def call(self, inst, **kwargs):
        # 1) 写入实参（inout 共享对象；input 数组深拷贝）
        kinds = {}
        for sec in ('in', 'inout', 'out'):
            for name, _d, _i in inst.defn['sections'][sec]:
                kinds[name] = sec
        for raw, val in kwargs.items():
            # 精确名优先（R 与 r、L 与 l 仅大小写不同，不可折叠）
            nm = raw if raw in kinds else raw
            if nm not in kinds:
                for k in kinds:           # 兜底：大小写不敏感
                    if k.lower() == raw.lower():
                        nm = k
                        break
            if kinds.get(nm) == 'inout':
                inst.vars[nm] = val                 # dict/list 直接共享
            elif isinstance(val, (list, dict)):
                inst.vars[nm] = copy.deepcopy(val)
            else:
                inst.vars[nm] = val
        # 2) 执行
        try:
            self._exec(inst.defn['body'], inst.vars, inst)
        except _Return:
            pass

    # ---------- 执行 ----------
    def _exec(self, stmts, frame, inst):
        for st in stmts:
            t = st[0]
            if t == 'assign':
                self._write(st[1], self._eval(st[2], frame), frame)
            elif t == 'if':
                done = False
                for cond, body in st[1]:
                    if self._eval(cond, frame):
                        self._exec(body, frame, inst)
                        done = True
                        break
                if not done and st[2] is not None:
                    self._exec(st[2], frame, inst)
            elif t == 'for':
                lo = int(self._eval(st[2], frame))
                hi = int(self._eval(st[3], frame))
                step = int(self._eval(st[4], frame))
                if step > 0:
                    cur, guard = lo, lambda c: c <= hi
                else:
                    cur, guard = lo, lambda c: c >= hi
                while guard(cur):
                    frame[st[1]] = cur
                    try:
                        self._exec(st[5], frame, inst)
                    except _Exit:
                        break
                    cur += step
            elif t == 'exit':
                raise _Exit()
            elif t == 'return':
                raise _Return()
            elif t == 'fbcall':
                self._fbcall(st[1], st[2], frame)

    def _fbcall(self, name_node, args, frame):
        callee = self._read(name_node, frame)
        kwargs = {}
        for nm, e in args:
            kwargs[nm] = self._eval(e, frame)
        self.call(callee, **kwargs)

    def _eval(self, node, frame):
        t = node[0]
        if t == 'num':
            return node[1]
        if t == 'bool':
            return node[1]
        if t == 'var':
            return frame[node[1]]
        if t == 'field':
            obj = self._eval(node[1], frame)
            if isinstance(obj, FBInstance):
                return obj.vars[node[2]]
            if not isinstance(obj, dict):
                raise RuntimeError('成员访问 %s.%s 的宿主不是结构/FB: %r'
                                   % (node[1], node[2], obj))
            return obj[node[2]]
        if t == 'index':
            obj = self._eval(node[1], frame)
            for ix in node[2][:-1] or []:
                obj = obj[int(self._eval(ix, frame))]
            return obj[int(self._eval(node[2][-1], frame))]
        if t == 'unop':
            v = self._eval(node[2], frame)
            return (not v) if node[1] == 'NOT' else -v
        if t == 'binop':
            op = node[1]
            if op == 'AND':
                return self._eval(node[2], frame) and self._eval(node[3], frame)
            if op == 'OR':
                return self._eval(node[2], frame) or self._eval(node[3], frame)
            a = self._eval(node[2], frame)
            b = self._eval(node[3], frame)
            if op == 'MOD':
                return int(a) % int(b)
            cmpmap = {'=': lambda: a == b, 'NEQ': lambda: a != b,
                      '<': lambda: a < b, '>': lambda: a > b,
                      'LE': lambda: a <= b, 'GE': lambda: a >= b}
            if op in cmpmap:
                return cmpmap[op]()
            if op == '/':
                if b == 0:
                    raise ZeroDivisionError(
                        'ST 除零: %r  左值=%r' % (node, a))
                return a / b
            return {'+': a + b, '-': a - b, '*': a * b}[op]
        if t == 'call':
            return self._builtin(node, frame)
        raise RuntimeError('未知表达式节点 %r' % (t,))

    def _read(self, node, frame):
        """取左值对应的容器对象（field/index 末端容器）。"""
        return self._eval(node, frame)

    def _write(self, node, value, frame):
        t = node[0]
        if t == 'var':
            frame[node[1]] = value
            return
        if t == 'field':
            obj = self._eval(node[1], frame)
            target = obj.vars if isinstance(obj, FBInstance) else obj
            target[node[2]] = value
            return
        if t == 'index':
            obj = self._eval(node[1], frame)
            for ix in node[2][:-1]:
                obj = obj[int(self._eval(ix, frame))]
            obj[int(self._eval(node[2][-1], frame))] = value
            return
        raise RuntimeError('非法左值 %r' % (t,))

    def _builtin(self, node, frame):
        fn_node, args = node[1], node[2]
        name = fn_node[1].upper()
        vals = [self._eval(e, frame) for _nm, e in args]
        if name == 'SQRT':
            return math.sqrt(vals[0])
        if name == 'SIN':
            return math.sin(vals[0])
        if name == 'COS':
            return math.cos(vals[0])
        if name == 'ASIN':
            return math.asin(vals[0])
        if name == 'ACOS':
            return math.acos(vals[0])
        if name == 'ATAN2':
            return math.atan2(vals[0], vals[1])
        if name == 'ABS':
            return abs(vals[0])
        if name == 'FLOOR':
            return math.floor(vals[0])
        if name == 'RAD':
            return math.radians(vals[0])
        if name == 'DEG':
            return math.degrees(vals[0])
        raise RuntimeError('未知内置函数 %s' % name)
