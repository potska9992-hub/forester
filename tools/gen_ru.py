#!/usr/bin/env python3
# Generates the Russian-language block (glyphs + dictionary + Disp class) and inserts it into pult.ino
# between the RU-BEGIN / RU-END markers.   usage: gen_ru.py <dict.py> <pult.ino>
import sys, re, importlib.util, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ru_glyphs
spec = importlib.util.spec_from_file_location("d", sys.argv[1]); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
D = m.D
# glyph table: 0..31 capitals, 32..63 lowercase
rows = []
for ch in ru_glyphs.UP + ru_glyphs.LO:
    g = ru_glyphs.G[ch]
    rows.append("  {" + ",".join("0x%02X" % int(r.replace('.', '0').replace('#', '1'), 2) for r in g) + "},  // " + ch)
ents = sorted(((k.lower(), v) for k, v in D.items() if k.lower() != v.lower() or True), key=lambda kv: kv[0].encode())
ents = [(k, v) for k, v in ents if k != v]            # identity entries are useless
first = {}
for i, (k, v) in enumerate(ents):
    first.setdefault(k.encode()[0], [i, 0])[1] += 1
def c(s): return '"' + s.replace('\\', '\\\\').replace('"', '\\"') + '"'
tab = ",\n".join("  {%s, %s}" % (c(k), c(v)) for k, v in ents)
idx = ["{0,0}"] * 128
for b, (s, n) in first.items():
    idx[b] = "{%d,%d}" % (s, n)
idxs = ",".join(idx)
tmpl = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "ru_disp.inc")).read()
block = tmpl.replace("@GLYPHS@", "\n".join(rows)).replace("@TAB@", tab).replace("@NTAB@", str(len(ents))).replace("@IDX@", idxs)
src = open(sys.argv[2], encoding="utf-8").read()
a = src.index("// ==== RU-BEGIN"); b = src.index("// ==== RU-END")
b = src.index("\n", b) + 1
out = src[:a] + block + src[b:]
open(sys.argv[2], "w", encoding="utf-8").write(out)
print("entries", len(ents), "bytes ~", sum(len(k) + len(v) + 12 for k, v in ents))
