#!/usr/bin/env python3
import csv
import html
import math
import os
import sys

NAMES = {1: "RED", 2: "YELLOW", 3: "GREEN", 4: "WATER"}
COLORS = {1: "#c62828", 2: "#d4a106", 3: "#2e7d32", 4: "#1f6fb4", "T1": "#c0392b", "T2": "#2c6fbb", "dT": "#6b3a2a"}
TAGS = ("#RUN", "#REPORT", "D,", "T,", "R,", "K,", "C,", "S,", "X,", "P,", "!,", "O,", "Q,", "H,")


def clean(raw):
    out = []
    for ln in raw:
        ln = ln.strip()
        for t in TAGS:
            i = ln.find(t)
            if i >= 0 and (i == 0 or ln[i - 1] in " \t]>|"):
                out.append(ln[i:])
                break
    return out


def pick_block(lines):
    begin = [i for i, l in enumerate(lines) if l.startswith("#REPORT BEGIN")]
    for b in reversed(begin):
        for e in range(b + 1, len(lines)):
            if lines[e].startswith("#REPORT END"):
                run = [i for i, l in enumerate(lines[:b]) if l.startswith("#RUN")]
                live = lines[run[-1]:b] if run else []
                return lines[b + 1:e], live, "stored report block"
            if lines[e].startswith("#REPORT BEGIN"):
                break
    run = [i for i, l in enumerate(lines) if l.startswith("#RUN")]
    if run:
        return lines[run[-1]:], lines[run[-1]:], "live log of the last run"
    return lines, lines, "whole log (no #RUN marker found)"


def num(s):
    try:
        v = float(s)
        return v if math.isfinite(v) else None
    except ValueError:
        return None


def parse(block, live, everything):
    d = {"D": [], "T": [], "R": {}, "K": {}, "C": [], "S": [], "X": {}, "P": {}, "TRIP": [], "O": {}, "Q": [], "H": {}}
    src_d = [l for l in block if l.startswith("D,")] or [l for l in live if l.startswith("D,")]
    for l in src_d:
        f = l.split(",")
        if len(f) >= 8 and num(f[1]) is not None:
            d["D"].append((num(f[1]), int(f[2]), f[3], num(f[4]), num(f[6]), num(f[7])))
    src_t = [l for l in block if l.startswith("T,")] or [l for l in live if l.startswith("T,")]
    for l in src_t:
        f = l.split(",")
        if len(f) >= 6:
            d["T"].append((num(f[1]), num(f[2]), num(f[3]), num(f[4]), int(f[5]) if f[5].strip().isdigit() else 0))
    for l in block + live:
        f = l.split(",")
        if l.startswith("R,") and len(f) >= 8:
            d["R"][int(f[1])] = dict(name=f[2], target=num(f[3]), ml=num(f[4]), err=num(f[5]), pulses=int(float(f[6])), secs=num(f[7]))
        elif l.startswith("H,") and len(f) >= 5 and f[1].isdigit():
            d["H"][int(f[1])] = (num(f[3]), num(f[4]))
        elif l.startswith("X,") and len(f) >= 3:
            d["X"][f[1]] = f[2:]
        elif l.startswith("P,") and len(f) >= 3:
            d["P"][f[1]] = num(f[2])
        elif l.startswith("S,") and len(f) >= 3:
            d["S"].append((num(f[1]), ",".join(f[2:])))
        elif l.startswith("!,TRIP") and len(f) >= 4:
            d["TRIP"].append((num(f[2]), ",".join(f[3:])))
    seen = set()
    for l in everything:
        f = l.split(",")
        if l.startswith("K,") and len(f) >= 5 and f[1].isdigit():
            d["K"][int(f[1])] = (f[2], num(f[3]), f[4])
        elif l.startswith("O,") and len(f) >= 3 and f[1] in ("TS1", "TS2"):
            d["O"][f[1]] = num(f[2])
        elif l.startswith("Q,") and len(f) >= 7 and f[1].isdigit():
            key = ("Q",) + tuple(f[1:7])
            if key not in seen:
                seen.add(key)
                d["Q"].append((int(f[1]), f[2], f[3], num(f[4]), num(f[5]), num(f[6])))
        elif l.startswith("C,") and len(f) >= 7 and f[1].isdigit():
            key = tuple(f[1:7])
            if key not in seen:
                seen.add(key)
                d["C"].append((int(f[1]), f[2], int(float(f[3])), int(float(f[4])), num(f[5]), num(f[6])))
    seen_s = []
    for s in d["S"]:
        if s not in seen_s:
            seen_s.append(s)
    d["S"] = seen_s
    return d


def nice_ticks(lo, hi, n=6):
    if hi <= lo:
        hi = lo + 1
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw))
    step = min((m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw), default=mag * 10)
    start = math.floor(lo / step) * step
    ticks = []
    v = start
    while v <= hi + step * 0.001:
        ticks.append(round(v, 10))
        v += step
    return ticks


def chart(series, xlabel, ylabel, hlines=(), w=860, h=340, ymin=None):
    pts = [p for s in series for p in s["pts"] if p[1] is not None]
    if not pts:
        return "<p class=muted>No data in the log for this chart.</p>"
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts] + [v for v, _, _ in hlines]
    x0, x1 = min(xs), max(xs)
    y0 = min(ys) if ymin is None else ymin
    y1 = max(ys)
    pad = (y1 - y0) * 0.08 or 1
    y0, y1 = (y0 - pad if ymin is None else y0), y1 + pad
    xt, yt = nice_ticks(x0, x1), nice_ticks(y0, y1)
    x0, x1 = min(x0, xt[0]), max(x1, xt[-1])
    y0, y1 = min(y0, yt[0]), max(y1, yt[-1])
    L, R, T, B = 64, 20, 16, 46
    X = lambda v: L + (v - x0) / ((x1 - x0) or 1) * (w - L - R)
    Y = lambda v: h - B - (v - y0) / ((y1 - y0) or 1) * (h - T - B)
    o = [f'<svg viewBox="0 0 {w} {h}" role="img" class="chart" xmlns="http://www.w3.org/2000/svg">']
    for t in yt:
        o.append(f'<line x1="{L}" x2="{w-R}" y1="{Y(t):.1f}" y2="{Y(t):.1f}" class="grid"/>')
        o.append(f'<text x="{L-8}" y="{Y(t)+4:.1f}" text-anchor="end" class="tick">{t:g}</text>')
    for t in xt:
        o.append(f'<text x="{X(t):.1f}" y="{h-B+18}" text-anchor="middle" class="tick">{t:g}</text>')
    o.append(f'<line x1="{L}" x2="{w-R}" y1="{h-B}" y2="{h-B}" class="axis"/>')
    o.append(f'<text x="{(L+w-R)/2}" y="{h-8}" text-anchor="middle" class="lab">{html.escape(xlabel)}</text>')
    o.append(f'<text x="14" y="{(T+h-B)/2}" text-anchor="middle" class="lab" transform="rotate(-90 14 {(T+h-B)/2})">{html.escape(ylabel)}</text>')
    for v, lab, col in hlines:
        o.append(f'<line x1="{L}" x2="{w-R}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" stroke="{col}" stroke-dasharray="6 5" stroke-width="1.5"/>')
        o.append(f'<text x="{w-R-4}" y="{Y(v)-6:.1f}" text-anchor="end" class="tick" fill="{col}">{html.escape(lab)}</text>')
    for s in series:
        seg, segs = [], []
        for x, y in s["pts"]:
            if y is None:
                if seg:
                    segs.append(seg)
                seg = []
            else:
                seg.append(f"{X(x):.1f},{Y(y):.1f}")
        if seg:
            segs.append(seg)
        for sg in segs:
            o.append(f'<polyline points="{" ".join(sg)}" fill="none" stroke="{s["color"]}" stroke-width="2" stroke-linejoin="round"/>')
    o.append("</svg>")
    leg = "".join(f'<span class="key"><i style="background:{s["color"]}"></i>{html.escape(s["name"])}</span>' for s in series)
    return f'<div class="legend">{leg}</div>' + "".join(o)


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def fmt(v, p=1, sign=False):
    if v is None:
        return "–"
    return f"{v:+.{p}f}" if sign else f"{v:.{p}f}"


def main():
    if len(sys.argv) < 2:
        print("usage: python3 report.py <monitor log> [output folder]")
        sys.exit(1)
    src = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + "_report"
    os.makedirs(out, exist_ok=True)
    with open(src, errors="replace") as f:
        lines = clean(f.readlines())
    block, live, how = pick_block(lines)
    d = parse(block, live, lines)

    flush = [(t, ml, r, duty) for t, ln, ph, duty, ml, r in d["D"] if ln == 4]
    sm = []
    j = 0
    for i, (t, ml, r, du) in enumerate(flush):
        while j < i and t - flush[j][0] > 2.0:
            j += 1
        dt = t - flush[j][0]
        sm.append((t, ml, round((ml - flush[j][1]) / dt, 2) if dt > 0.2 else r, du))
    flush = sm
    write_csv(os.path.join(out, "flush_run.csv"), ["t_s", "volume_mL", "rate_mL_s", "pump_pct"], flush)
    calrows = [(i, d["K"][i][0], d["K"][i][2], d["K"][i][1]) for i in sorted(d["K"])]
    write_csv(os.path.join(out, "calibration_k.csv"), ["line", "name", "sensor", "k_pulses_per_mL"], calrows)
    write_csv(os.path.join(out, "calibration_runs.csv"), ["line", "name", "pulses", "run_ms", "mL_measured", "k"],
              [c for c in d["C"]])
    write_csv(os.path.join(out, "calibration_temp.csv"), ["probe", "method", "raw_C", "reference_C", "offset_C"],
              [(p, m, r, ref, o) for i, p, m, r, ref, o in d["Q"]] or [(p, "in use", "", "", o) for p, o in sorted(d["O"].items())])
    write_csv(os.path.join(out, "dt_trend.csv"), ["t_s", "TS1_C", "TS2_C", "dT_C", "cooling_pump_P4"], d["T"])
    dose_rows = [(i, r["name"], r["target"], r["ml"], r["err"], r["pulses"], r["secs"]) for i, r in sorted(d["R"].items())]
    write_csv(os.path.join(out, "dosed_volumes.csv"), ["line", "name", "target_mL", "metered_mL", "error_mL", "pulses", "time_s"], dose_rows)
    write_csv(os.path.join(out, "dose_traces.csv"), ["t_s", "line", "phase", "pump_pct", "volume_mL", "rate_mL_s"], d["D"])

    tgt_flush = d["R"].get(4, {}).get("target") or d["P"].get("target4") or 2000.0
    dtt = d["P"].get("dt_target") or 10.0
    sec = []
    fr = d["R"].get(4)
    t_first = flush[0][0] if flush else 0
    sec.append("<h2>1 · Flush run</h2>")
    if fr:
        sec.append(f'<div class="kpis"><div><b>{fmt(fr["ml"])} mL</b><span>totalised on FT1</span></div>'
                   f'<div><b>{fmt(fr["target"],0)} mL</b><span>target</span></div>'
                   f'<div><b>{fmt(fr["err"],1,True)} mL</b><span>error</span></div>'
                   f'<div><b>{fmt(fr["secs"])} s</b><span>pump on to valves shut</span></div></div>')
        sec.append("<p>Water from the flush tank through SV4, tee TA, P1 (red pump) and FT1 into the 2 L jug standing in the mixing tank. P1 switched off by itself at the target, then SV4 closed.</p>")
    sec.append(chart([{"name": "Flushed volume (mL)", "color": COLORS[4], "pts": [(t - t_first, ml) for t, ml, r, du in flush]}],
                     "time from pump start (s)", "volume (mL)", [(tgt_flush, f"target {tgt_flush:g} mL", "#888")], ymin=0))
    sec.append(chart([{"name": "Flow rate (mL/s)", "color": COLORS[4], "pts": [(t - t_first, r) for t, ml, r, du in flush]},
                      {"name": "Pump duty / 10 (%)", "color": "#999", "pts": [(t - t_first, (du or 0) / 10) for t, ml, r, du in flush]}],
                     "time from pump start (s)", "mL/s", ymin=0, h=240))

    sec.append("<h2>2 · Calibration table</h2>")
    if calrows:
        sec.append("<table><tr><th>Line</th><th>Flow sensor</th><th>k (pulses/mL)</th><th>mL per pulse</th></tr>" + "".join(
            f"<tr><td>{i} {html.escape(n)}</td><td>{html.escape(s)}</td><td>{fmt(k,4)}</td><td>{fmt(1/k if k else None,4)}</td></tr>" for i, n, s, k in calrows) + "</table>")
    if d["C"]:
        sec.append("<p>Calibration runs logged with <code>calrun</code> + <code>k</code>:</p><table><tr><th>Line</th><th>Pulses</th><th>Run (s)</th><th>Measured (mL)</th><th>k</th></tr>" + "".join(
            f"<tr><td>{l} {html.escape(n)}</td><td>{p}</td><td>{ms/1000:.1f}</td><td>{fmt(ml)}</td><td>{fmt(k,4)}</td></tr>" for l, n, p, ms, ml, k in d["C"]) + "</table>")
    else:
        sec.append("<p class=muted>No calibration runs in this log, only the k values in use. Values from Day 1 calibration.</p>")

    sec.append("<h3>Temperature probes (DS18B20)</h3>")
    if d["Q"]:
        sec.append("<table><tr><th>Probe</th><th>Method</th><th>Raw (°C)</th><th>Reference (°C)</th><th>Offset (°C)</th></tr>" + "".join(
            f"<tr><td>{html.escape(p)}</td><td>{html.escape(m)}</td><td>{fmt(r,2)}</td><td>{fmt(ref,2)}</td><td>{fmt(o,2,True)}</td></tr>" for i, p, m, r, ref, o in d["Q"]) + "</table>")
    if d["O"]:
        sec.append("<p>Offsets applied to every reading in this run: " + ", ".join(f"{p} {fmt(o,2,True)} °C" for p, o in sorted(d["O"].items())) + ". Raw DS18B20 accuracy is ±0.5 °C per probe, so without offsets ΔT could be off by up to 1 °C.</p>")
    if not d["Q"] and not d["O"]:
        sec.append("<p class=muted>No probe calibration rows in this log.</p>")

    sec.append("<h2>3 · ΔT trend</h2>")
    xr = d["X"].get("dt_reached")
    if xr:
        sec.append(f'<div class="kpis"><div><b>{xr[0]} °C</b><span>ΔT at hand-over</span></div><div><b>{xr[1]} s</b><span>time in Step 2</span></div><div><b>{dtt:g} °C</b><span>required</span></div></div>')
    tt = d["T"]
    sec.append(chart([{"name": "TS1 cooled yellow tank (°C)", "color": COLORS["T1"], "pts": [(t, a) for t, a, b, dt, hh in tt]},
                      {"name": "TS2 reference tank (°C)", "color": COLORS["T2"], "pts": [(t, b) for t, a, b, dt, hh in tt]}],
                     "time from START (s)", "°C"))
    sec.append(chart([{"name": "ΔT = |TS1 − TS2| (°C)", "color": COLORS["dT"], "pts": [(t, dt) for t, a, b, dt, hh in tt]},
                      {"name": "cooling pump P4 on (1/0)", "color": "#e67e22", "pts": [(t, hh) for t, a, b, dt, hh in tt]}],
                     "time from START (s)", "°C", [(dtt, f"ΔT {dtt:g} °C", "#888")], ymin=0, h=260))

    sec.append("<h2>4 · Dosed volumes</h2>")
    cols = [r for i, r in sorted(d["R"].items()) if i in (1, 2, 3)]
    tot = sum(r["ml"] or 0 for r in cols)
    if cols:
        sec.append("<table><tr><th>Colour</th><th>Target (mL)</th><th>Metered (mL)</th><th>Error (mL)</th><th>Error (%)</th><th>Share of batch</th><th>Time (s)</th></tr>" + "".join(
            f"<tr><td>{html.escape(r['name'])}</td><td>{fmt(r['target'])}</td><td>{fmt(r['ml'])}</td><td>{fmt(r['err'],1,True)}</td>"
            f"<td>{fmt((r['err'] or 0)*100/(r['target'] or 1),2,True)}</td><td>{fmt((r['ml'] or 0)*100/(tot or 1),1)} %</td><td>{fmt(r['secs'])}</td></tr>" for r in cols)
            + f"<tr><th>Total</th><th>{fmt(sum(r['target'] or 0 for r in cols))}</th><th>{fmt(tot)}</th><th colspan=4></th></tr></table>")
    for ln, (hold, metered) in sorted(d["H"].items()):
        sec.append(f"<p>{NAMES.get(ln, ln)}: FT1 metered {fmt(metered)} mL in total. The first {fmt(hold)} mL was the flush water still standing in the red line after Step 1, "
                   f"so the table shows the red that reached the mixing tank ({fmt(metered)} − {fmt(hold)} mL).</p>")
    dser = []
    for ln in (1, 2, 3):
        pts = [(t, ml) for t, l, ph, du, ml, r in d["D"] if l == ln]
        if pts:
            t0 = pts[0][0]
            dser.append({"name": f"{NAMES[ln]} (mL)", "color": COLORS[ln], "pts": [(t - t0, v) for t, v in pts]})
    sec.append(chart(dser, "time from start of each dose (s)", "volume (mL)", ymin=0))

    if d["S"] or d["TRIP"]:
        sec.append("<h2>Sequence log</h2><table><tr><th>t (s)</th><th>Event</th></tr>" + "".join(
            f"<tr><td>{fmt(t)}</td><td>{html.escape(e)}</td></tr>" for t, e in d["S"]) + "".join(
            f"<tr><td>{fmt(t)}</td><td><b>TRIP</b> {html.escape(e)}</td></tr>" for t, e in d["TRIP"]) + "</table>")

    title = "Team 13 · Day 2 run report"
    page = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Day 2 Run Report</title><style>
:root{{--ink:#16181d;--ink2:#555b66;--rule:#d9dce1;--soft:#f3f5f8;--bg:#fff}}
@media (prefers-color-scheme: dark){{:root:not([data-theme="light"]){{--ink:#e8eaee;--ink2:#a9afba;--rule:#3a3f48;--soft:#22262d;--bg:#16181d}}}}
:root[data-theme="dark"]{{--ink:#e8eaee;--ink2:#a9afba;--rule:#3a3f48;--soft:#22262d;--bg:#16181d}}
body{{margin:0;background:var(--bg);color:var(--ink);font:15px/1.5 "Segoe UI",Roboto,Arial,sans-serif}}
main{{max-width:900px;margin:0 auto;padding:28px 16px 60px}}
h1{{margin:0 0 4px;font-size:26px}} h2{{margin:34px 0 10px;font-size:20px;border-top:2px solid var(--ink);padding-top:10px}}
.muted,.sub{{color:var(--ink2)}} table{{border-collapse:collapse;width:100%;font-size:14px;margin:10px 0;display:block;overflow-x:auto}}
th,td{{border:1px solid var(--rule);padding:5px 8px;text-align:left}} th{{background:var(--soft)}}
.kpis{{display:flex;flex-wrap:wrap;gap:10px;margin:10px 0}} .kpis div{{background:var(--soft);border-radius:8px;padding:10px 14px;min-width:150px}}
.kpis b{{display:block;font-size:20px}} .kpis span{{color:var(--ink2);font-size:13px}}
.chart{{width:100%;height:auto;margin:4px 0 14px}} .chart .grid{{stroke:var(--rule)}} .chart .axis{{stroke:var(--ink2)}}
.chart .tick{{font-size:12px;fill:var(--ink2)}} .chart .lab{{font-size:13px;fill:var(--ink)}}
.legend{{display:flex;flex-wrap:wrap;gap:14px;font-size:13px;color:var(--ink2)}} .key i{{display:inline-block;width:14px;height:3px;margin-right:6px;vertical-align:middle}}
code{{background:var(--soft);padding:1px 4px;border-radius:3px}}
@media print{{h2{{break-before:auto}} .chart,table{{break-inside:avoid}}}}
</style></head><body><main>
<h1>{title}</h1><p class="sub">Source: {html.escape(os.path.basename(src))} · {html.escape(how)} · CSV files next to this page</p>
{''.join(sec)}
</main></body></html>"""
    with open(os.path.join(out, "report.html"), "w") as f:
        f.write(page)
    print(f"wrote {out}/report.html and 7 CSV files")
    if fr:
        print(f"flush {fr['ml']} mL, " + ", ".join(f"{r['name']} {r['ml']} mL" for r in cols))


if __name__ == "__main__":
    main()
