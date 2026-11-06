#!/usr/bin/env python3
import sys
W, H = 2000, 1360
out = []
def a(s): out.append(s)

PIPE = 'stroke="#111" stroke-width="3.5" fill="none"'
def line(x1, y1, x2, y2, extra=''):
    a(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" {PIPE} {extra}/>')
def poly(pts, extra=''):
    p = ' '.join(f'{x},{y}' for x, y in pts)
    a(f'<polyline points="{p}" {PIPE} {extra}/>')
def text(x, y, s, size=18, anchor='middle', weight='normal', fill='#111', style=''):
    a(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" font-weight="{weight}" fill="{fill}" style="{style}">{s}</text>')
def arrow(x, y, d='r', c='#111'):
    s = 9
    pts = {'r': [(x - s, y - s), (x + s, y), (x - s, y + s)],
           'l': [(x + s, y - s), (x - s, y), (x + s, y + s)],
           'd': [(x - s, y - s), (x, y + s), (x + s, y - s)],
           'u': [(x - s, y + s), (x, y - s), (x + s, y + s)]}[d]
    p = ' '.join(f'{px},{py}' for px, py in pts)
    a(f'<polygon points="{p}" fill="{c}"/>')

def bowtie(x, y, vertical=False, s=16):
    if not vertical:
        a(f'<polygon points="{x-s},{y-s*0.75} {x+s},{y+s*0.75} {x+s},{y-s*0.75} {x-s},{y+s*0.75}" fill="#fff" stroke="#111" stroke-width="2.5"/>')
    else:
        a(f'<polygon points="{x-s*0.75},{y-s} {x+s*0.75},{y+s} {x-s*0.75},{y+s} {x+s*0.75},{y-s}" fill="#fff" stroke="#111" stroke-width="2.5"/>')

def ball_valve(x, y, tag):
    bowtie(x, y)
    a(f'<circle cx="{x}" cy="{y}" r="6" fill="#111"/>')
    text(x, y + 40, tag, 16, weight='bold')
    text(x, y + 58, 'hand, open', 13, fill='#555')

def sol_valve(x, y, tag, pin, vertical=False, label_side='above'):
    bowtie(x, y, vertical)
    if not vertical:
        a(f'<line x1="{x}" y1="{y}" x2="{x}" y2="{y-30}" stroke="#111" stroke-width="2.5"/>')
        a(f'<rect x="{x-15}" y="{y-58}" width="30" height="28" fill="#fff" stroke="#111" stroke-width="2.5"/>')
        text(x, y - 37, 'S', 18, weight='bold')
        text(x, y + 40, tag, 16, weight='bold')
        text(x, y + 58, f'NC · {pin}', 13, fill='#555')
    else:
        a(f'<line x1="{x}" y1="{y}" x2="{x-30}" y2="{y}" stroke="#111" stroke-width="2.5"/>')
        a(f'<rect x="{x-58}" y="{y-14}" width="28" height="28" fill="#fff" stroke="#111" stroke-width="2.5"/>')
        text(x - 44, y + 7, 'S', 18, weight='bold')
        text(x + 30, y - 2, tag, 16, anchor='start', weight='bold')
        text(x + 30, y + 16, f'NC · {pin}', 13, anchor='start', fill='#555')

def pump(x, y, tag, sub):
    r = 26
    a(f'<circle cx="{x}" cy="{y}" r="{r}" fill="#fff" stroke="#111" stroke-width="2.5"/>')
    a(f'<line x1="{x}" y1="{y-r}" x2="{x+r+4}" y2="{y-r}" stroke="#111" stroke-width="2.5"/>')
    a(f'<polygon points="{x-10},{y-12} {x+14},{y} {x-10},{y+12}" fill="#111"/>')
    text(x, y + 50, tag, 16, weight='bold')
    text(x, y + 68, sub, 13, fill='#555')

def flow_sensor(x, y, tag, pin):
    a(f'<rect x="{x-14}" y="{y-11}" width="28" height="22" fill="#fff" stroke="#111" stroke-width="2.5"/>')
    a(f'<line x1="{x-8}" y1="{y-11}" x2="{x+8}" y2="{y+11}" stroke="#111" stroke-width="1.5"/>')
    by = y - 62
    a(f'<line x1="{x}" y1="{y-11}" x2="{x}" y2="{by+24}" stroke="#111" stroke-width="1.5" stroke-dasharray="5,4"/>')
    a(f'<circle cx="{x}" cy="{by}" r="24" fill="#fff" stroke="#111" stroke-width="2.5"/>')
    text(x, by - 3, 'FT', 15, weight='bold')
    text(x, by + 14, tag[-1], 15, weight='bold')
    text(x - 30, by + 5, pin, 13, anchor='end', fill='#555')

def tank(x, y, w, h, tag, name, fill, sub=''):
    a(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="#fff" stroke="#111" stroke-width="3"/>')
    a(f'<rect x="{x+4}" y="{y+h*0.5}" width="{w-8}" height="{h*0.5-4}" rx="5" fill="{fill}" opacity="0.85"/>')
    text(x + w / 2, y + 24, tag, 17, weight='bold')
    text(x + w / 2, y + 44, name, 15)
    if sub:
        text(x + w / 2, y + h + 20, sub, 13, fill='#555')

def ball_valve_v(x, y, tag, sub):
    bowtie(x, y, vertical=True)
    a(f'<circle cx="{x}" cy="{y}" r="6" fill="#111"/>')
    text(x + 26, y - 2, tag, 16, anchor='start', weight='bold')
    text(x + 26, y + 16, sub, 13, anchor='start', fill='#555')

def nozzle(x, y, up=True, lab=None, anchor='start', dx=8):
    y2 = y - 14 if up else y + 14
    a(f'<rect x="{x-7}" y="{min(y, y2)}" width="14" height="14" fill="#fff" stroke="#111" stroke-width="2"/>')
    if lab:
        text(x + dx, (y2 - 4) if up else (y2 + 14), lab, 12, anchor=anchor, fill='#555')

def bubble(x, y, l1, l2, pin, tx, ty):
    a(f'<line x1="{x}" y1="{y}" x2="{tx}" y2="{ty}" stroke="#111" stroke-width="1.5" stroke-dasharray="5,4"/>')
    a(f'<circle cx="{x}" cy="{y}" r="24" fill="#fff" stroke="#111" stroke-width="2.5"/>')
    text(x, y - 3, l1, 15, weight='bold')
    text(x, y + 14, l2, 15, weight='bold')
    text(x, y + 42, pin, 12, fill='#555')

def heater(x, y, w=60):
    pts = [(x, y)]
    n = 6
    for i in range(1, n + 1):
        pts.append((x + w * i / n, y - 10 if i % 2 else y + 10))
    p = ' '.join(f'{px:.0f},{py}' for px, py in pts)
    a(f'<polyline points="{p}" fill="none" stroke="#c0392b" stroke-width="3"/>')

a(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="DejaVu Sans, Arial, sans-serif">')
a(f'<rect width="{W}" height="{H}" fill="#fff"/>')
a(f'<rect x="10" y="10" width="{W-20}" height="{H-20}" fill="none" stroke="#111" stroke-width="2"/>')
a('<g transform="translate(0,90)">')

XT, TW, TH = 110, 170, 120
XOUT = XT + TW - 35
XBV, XSVS, XTA, XP, XFT, XSV = 325, 425, 482, 545, 665, 835
YW, YR, YG, YY = 170, 370, 560, 750

def tank_row(y, tt, nm, fill, bv=True):
    ty = y - 30 - TH
    tank(XT, ty, TW, TH, tt, nm, fill)
    nozzle(XOUT, y - 30, up=False)
    poly([(XOUT, y - 16), (XOUT, y), (XBV - 16, y)])
    return ty

ty4 = tank_row(YW, 'T-4', 'FLUSH WATER', '#cfe8ff')
bubble(52, ty4 + 60, 'TT', '2', 'TS2 · U7 G13', XT + 20, ty4 + 80)
ball_valve(XBV, YW, 'BV-F')
line(XBV + 16, YW, XSVS - 16, YW)
sol_valve(XSVS, YW, 'SV4', 'U2 G19')
poly([(XSVS + 16, YW), (XTA, YW), (XTA, YR)])
arrow(XTA, 270, 'd')
text(XTA + 12, YW + 28, 'water', 12, anchor='start', fill='#555')

ty1 = tank_row(YR, 'T-1', 'RED · 60%', '#d7263d')
ball_valve(XBV, YR, 'BV-1')
line(XBV + 16, YR, XSVS - 16, YR)
sol_valve(XSVS, YR, 'SV1', 'U2 G23')
line(XSVS + 16, YR, XP - 26, YR)
a(f'<circle cx="{XTA}" cy="{YR}" r="6" fill="#111"/>')
text(XTA - 8, YR + 26, 'TA', 13, anchor='end', fill='#333', weight='bold')
pump(XP, YR, 'P1', 'PWM_1 · U2 G32 · DRV1')
line(XP + 26, YR, XFT - 14, YR)
flow_sensor(XFT, YR, 'FT1', 'FS1 · U7 G33')
poly([(XFT + 14, YR), (1090, YR), (1090, 440), (1370, 440), (1370, 506)])
arrow(800, YR)
text(760, YR - 14, 'red / flush water', 12, fill='#555')

tyg = tank_row(YG, 'T-3', 'GREEN · 30%', '#2e9e5b')
ball_valve(XBV, YG, 'BV-3')
line(XBV + 16, YG, XP - 26, YG)
arrow(XBV + 80, YG)
pump(XP, YG, 'P3', 'PWM_3 · U2 G25 · DRV2')
line(XP + 26, YG, XFT - 14, YG)
flow_sensor(XFT, YG, 'FT3', 'FS3 · U7 G26')
line(XFT + 14, YG, XSV - 16, YG)
sol_valve(XSV, YG, 'SV3', 'U2 G21')
poly([(XSV + 16, YG), (1010, YG), (1010, 470), (1300, 470), (1300, 506)])
arrow(940, YG)

tyy = tank_row(YY, 'T-2', 'YELLOW · cooled', '#f2c14e')
nozzle(XT + 50, tyy, up=True)
bubble(52, tyy + 60, 'TT', '1', 'TS1 · U7 G32', XT + 20, tyy + 80)
ball_valve(XBV, YY, 'BV-2')
line(XBV + 16, YY, XP - 26, YY)
XTC = 430
a(f'<circle cx="{XTC}" cy="{YY}" r="6" fill="#111"/>')
text(XTC, YY - 14, 'TC', 13, fill='#333', weight='bold')
pump(XP, YY, 'P2', 'PWM_2 · U2 G33 · DRV1')
line(XP + 26, YY, XFT - 14, YY)
flow_sensor(XFT, YY, 'FT2', 'FS2 · U7 G25')
line(XFT + 14, YY, XSV - 16, YY)
sol_valve(XSV, YY, 'SV2', 'U2 G22')
poly([(XSV + 16, YY), (1050, YY), (1050, 500), (1230, 500), (1230, 506)])
arrow(940, YY)

YL = 900
poly([(XTC, YY), (XTC, YL), (XP - 26, YL)])
arrow(XTC, 840, 'd')
pump(XP, YL, 'P4', 'PWM_4 · U2 G26 · DRV2')
line(XP + 26, YL, XSV - 16, YL)
sol_valve(XSV, YL, 'SV5', 'U2 G18')
BX, BY, BW, BH = 920, 850, 200, 150
line(XSV + 16, YL, BX + 30, YL)
arrow(890, YL)
a(f'<rect x="{BX}" y="{BY}" width="{BW}" height="{BH}" rx="8" fill="#fff" stroke="#111" stroke-width="3"/>')
a(f'<rect x="{BX+4}" y="{BY+30}" width="{BW-8}" height="{BH-34}" rx="5" fill="#e3f2fb"/>')
for k in range(7):
    a(f'<ellipse cx="{BX+60+k*16}" cy="{BY+90}" rx="9" ry="34" fill="none" stroke="#b5651d" stroke-width="3"/>')
poly([(BX + 30, YL), (BX + 52, YL)])
text(BX + BW / 2, BY + 22, 'T-6 ICE BATH 4.2 L', 14, weight='bold')
text(BX + BW / 2, BY + BH + 20, 'CL-1 copper coil', 13, fill='#555')
YRET = 1030
poly([(BX + 172, BY + 90), (BX + BW + 30, BY + 90), (BX + BW + 30, YRET), (90, YRET), (90, tyy - 30), (XT + 50, tyy - 30), (XT + 50, tyy - 14)])
arrow(600, YRET, 'l')
arrow(90, 800, 'u')
text(600, YRET + 22, 'cold yellow back into T-2 top hole (end above liquid)', 12, fill='#555')

MX, MY, MW, MH = 1180, 520, 240, 280
tank(MX, MY, MW, MH, 'T-5', 'MIXING TANK', '#6b3a2a', '')
for hx in (1230, 1300, 1370):
    nozzle(hx, MY, up=True)
    arrow(hx, MY - 22, 'd')
a(f'<path d="M 1335 590 L 1342 670 L 1398 670 L 1405 590" fill="#e9f3ff" stroke="#1f5fb4" stroke-width="2" stroke-dasharray="6,4"/>')
text(1370, 630, 'J-1', 13, weight='bold', fill='#1f5fb4')
text(1370, 646, 'jug', 11, fill='#1f5fb4')
text(1300, MY + MH + 22, '10 L · batch 3,000 mL · 3 top holes Y | G | R', 12, fill='#555')
text(1300, MY + MH + 40, 'J-1 2 L jug stands under the red outlet in Step 1 only', 12, fill='#1f5fb4')

a('<rect x="40" y="1060" width="1110" height="180" rx="6" fill="#fafafa" stroke="#111" stroke-width="2" stroke-dasharray="8,5"/>')
text(60, 1090, 'UC-1 CONTROL PCB (12 VDC) · one START press', 18, anchor='start', weight='bold')
for i, s in enumerate([
    'Step 1: SV4 open, P1 pumps flush water through TA and FT1 into J-1, stops at 2.0 L. Lift J-1 out.',
    'Step 2: SV5 open, P4 circulates yellow through CL-1 in the ice bath until |TT-1 − TT-2| ≥ 10 °C.',
    'Step 3: Yellow 300 (P2/SV2) → Green 900 (P3/SV3) → Red 1,800 (SV1/P1). Then all off.',
    'Interlocks trip all pumps: no flow, empty tank, mixing tank / jug full, link lost, no cooling, START.',
    'TT-1/TT-2 DS18B20 probes: red = VDD, black = GND, brown = DATA → PCB TS1 (U7 G32) / TS2 (U7 G13).',
    'SV1 and SV4 never open together. SV6 and heater output H1 not used.',
]):
    text(60, 1118 + i * 21, s, 14, anchor='start')

LX, LY = 1180, 860
a(f'<rect x="{LX}" y="{LY}" width="340" height="380" rx="6" fill="#fff" stroke="#111" stroke-width="2"/>')
text(LX + 16, LY + 28, 'LEGEND', 17, anchor='start', weight='bold')
def leg(i, draw, label):
    yy = LY + 56 + i * 33
    draw(LX + 45, yy)
    text(LX + 90, yy + 5, label, 14, anchor='start')
leg(0, lambda x, y: (bowtie(x, y, s=12), a(f'<circle cx="{x}" cy="{y}" r="5" fill="#111"/>')), 'Ball valve (hand, left open)')
def _sv(x, y):
    bowtie(x, y + 6, s=11)
    a(f'<line x1="{x}" y1="{y+6}" x2="{x}" y2="{y-6}" stroke="#111" stroke-width="2"/>')
    a(f'<rect x="{x-8}" y="{y-18}" width="16" height="12" fill="#fff" stroke="#111" stroke-width="2"/>')
leg(1, _sv, 'Solenoid valve, normally closed')
def _p(x, y):
    a(f'<circle cx="{x}" cy="{y}" r="14" fill="#fff" stroke="#111" stroke-width="2.5"/>')
    a(f'<polygon points="{x-6},{y-7} {x+8},{y} {x-6},{y+7}" fill="#111"/>')
leg(2, _p, 'Diaphragm pump, PWM speed')
def _b(t):
    def f(x, y):
        a(f'<circle cx="{x}" cy="{y}" r="14" fill="#fff" stroke="#111" stroke-width="2.5"/>')
        text(x, y + 4, t, 11, weight='bold')
    return f
leg(3, _b('FT'), 'Flow transmitter (pulse)')
leg(4, _b('TT'), 'Temperature probe DS18B20')
leg(5, lambda x, y: [a(f'<ellipse cx="{x-16+k*10}" cy="{y}" rx="5" ry="13" fill="none" stroke="#b5651d" stroke-width="2.5"/>') for k in range(4)], 'Copper cooling coil')
leg(6, lambda x, y: a(f'<circle cx="{x}" cy="{y}" r="6" fill="#111"/>'), 'Tee junction')
leg(7, lambda x, y: (line(x - 25, y, x + 25, y), arrow(x + 5, y)), 'Process line 8×5 mm + flow')
leg(8, lambda x, y: a(f'<line x1="{x-25}" y1="{y}" x2="{x+25}" y2="{y}" stroke="#111" stroke-width="1.5" stroke-dasharray="5,4"/>'), 'Signal line')
leg(9, lambda x, y: a(f'<rect x="{x-7}" y="{y-7}" width="14" height="14" fill="#fff" stroke="#111" stroke-width="2"/>'), 'Tank hole (bulkhead)')

TX, TY = 1550, 0
a(f'<rect x="{TX}" y="{TY}" width="420" height="1240" rx="6" fill="#fff" stroke="#111" stroke-width="2"/>')
text(TX + 16, TY + 30, 'TAG LIST', 18, anchor='start', weight='bold')
tags = [
    ('T-1', 'Red tank 10 L · 1 bottom hole'),
    ('T-2', 'Yellow tank 10 L · top + bottom, cooled'),
    ('T-3', 'Green tank 10 L · 1 bottom hole'),
    ('T-4', 'Flush water 10 L · 1 bottom hole'),
    ('T-5', 'Mixing tank 10 L · 3 top holes'),
    ('T-6', 'Ice bath 4.2 L with coil CL-1'),
    ('J-1', 'Flush jug 2 L, in T-5 for Step 1'),
    (None, None),
    ('P1', 'Red + flush pump · G32 · DRV1'),
    ('P2', 'Yellow pump · G33 · DRV1'),
    ('P3', 'Green pump · G25 · DRV2'),
    ('P4', 'Cooling loop pump · G26 · DRV2'),
    (None, None),
    ('FT1', 'Red / water flow · FS1 · U7 G33'),
    ('FT2', 'Yellow flow · FS2 · U7 G25'),
    ('FT3', 'Green flow · FS3 · U7 G26'),
    ('TT-1', 'T-2 yellow temp · TS1 · U7 G32'),
    ('TT-2', 'T-4 water temp · TS2 · U7 G13'),
    (None, None),
    ('SV1', 'Red tank → TA · G23'),
    ('SV2', 'Yellow → T-5 · G22'),
    ('SV3', 'Green → T-5 · G21'),
    ('SV4', 'Flush water → TA · G19'),
    ('SV5', 'Cooling loop, P4 → coil · G18'),
    (None, None),
    ('BV-F', 'T-4 outlet isolation, open'),
    ('BV-1..3', 'Colour outlet isolation, open'),
    ('TA', 'Equal tee: water + red suction'),
    ('TC', 'Equal tee: yellow dose / loop'),
    ('CL-1', 'Copper coil, push-fit ends'),
    ('HS-1', 'START switch · U2 G14'),
]
yy = TY + 62
for t, d in tags:
    if t is None:
        a(f'<line x1="{TX+12}" y1="{yy-12}" x2="{TX+408}" y2="{yy-12}" stroke="#ccc" stroke-width="1"/>')
        yy += 8
        continue
    text(TX + 16, yy, t, 14, anchor='start', weight='bold')
    text(TX + 100, yy, d, 14, anchor='start')
    yy += 31
text(TX + 16, yy + 2, 'SV pins on ESP32 #1 (U2), HIGH = open.', 12, anchor='start', fill='#555')
text(TX + 16, yy + 20, 'Valve and flow-sensor arrows = flow direction.', 12, anchor='start', fill='#555')
a('</g>')

text(40, 46, 'P&amp;ID · Chocolate batch plant · Team 13 · Day 2 rev B (5 solenoid valves, ice-bath cooling)', 24, anchor='start', weight='bold')
text(40, 72, 'START → Step 1 flush 2.0 L water through the red line into J-1 → Step 2 cool yellow until ΔT ≥ 10 °C → Step 3 Yellow 300 → Green 900 → Red 1,800 mL → all off', 14, anchor='start', fill='#555')
a('</svg>')
open(sys.argv[1], 'w').write('\n'.join(out))
