"""Inline CSS + Canvas JS for the results dashboard (see build_dashboard.py).

Kept separate so the generator stays under the repo's 500-line limit. The
palette mirrors the figures' house style (blue/green/amber signals on a cool,
instrument-panel neutral) and is fully token-driven for light/dark theming.
"""

CSS = r"""
:root{
  --plane:#eef1f5; --surface:#ffffff; --surface-2:#f5f7fa; --plate:#fcfdfe;
  --ink:#111621; --ink-2:#495264; --muted:#7d8798; --line:#dde3ec;
  --accent:#1f6feb; --accent-soft:#e7f0fe;
  --good:#0a7d3a; --good-soft:#e2f4e8; --warn:#b06f00; --bad:#c62f2f; --bad-soft:#fbe6e6;
  --hero-1:#0d1526; --hero-2:#132743; --hero-ink:#eef3fb; --hero-ink2:#9db2d4;
  --mono:"Cascadia Code","SF Mono","JetBrains Mono",ui-monospace,Consolas,"Liberation Mono",monospace;
  --sans:"Segoe UI",system-ui,-apple-system,Roboto,"Helvetica Neue",Arial,sans-serif;
}
@media (prefers-color-scheme:dark){:root{
  --plane:#0b0e14; --surface:#131822; --surface-2:#171d29; --plate:#f7f9fb;
  --ink:#e9edf4; --ink-2:#a7b2c4; --muted:#6d7889; --line:#232b39;
  --accent:#4f97ff; --accent-soft:#12233d;
  --good:#3fca6b; --good-soft:#10301d; --warn:#e0a53a; --bad:#f0665f; --bad-soft:#331717;
  --hero-1:#080d17; --hero-2:#0f1f37; --hero-ink:#eef3fb; --hero-ink2:#9db2d4;
}}
:root[data-theme="light"]{
  --plane:#eef1f5; --surface:#ffffff; --surface-2:#f5f7fa; --plate:#fcfdfe;
  --ink:#111621; --ink-2:#495264; --muted:#7d8798; --line:#dde3ec;
  --accent:#1f6feb; --accent-soft:#e7f0fe;
  --good:#0a7d3a; --good-soft:#e2f4e8; --warn:#b06f00; --bad:#c62f2f; --bad-soft:#fbe6e6;
}
:root[data-theme="dark"]{
  --plane:#0b0e14; --surface:#131822; --surface-2:#171d29; --plate:#f7f9fb;
  --ink:#e9edf4; --ink-2:#a7b2c4; --muted:#6d7889; --line:#232b39;
  --accent:#4f97ff; --accent-soft:#12233d;
  --good:#3fca6b; --good-soft:#10301d; --warn:#e0a53a; --bad:#f0665f; --bad-soft:#331717;
}
*{box-sizing:border-box}
.page{background:var(--plane);color:var(--ink);font-family:var(--sans);
  line-height:1.58;-webkit-font-smoothing:antialiased;overflow-x:hidden}
h1,h2,h3,h4{text-wrap:balance;line-height:1.15;letter-spacing:-.015em;margin:0}
p{margin:0 0 1em}
code{font-family:var(--mono);font-size:.86em}
.eyebrow,.kicker{font-family:var(--mono);text-transform:uppercase;
  letter-spacing:.14em;font-size:.72rem;font-weight:600}

/* ---- hero ---- */
.hero{background:linear-gradient(155deg,var(--hero-1),var(--hero-2));
  color:var(--hero-ink);padding:clamp(2.4rem,5vw,4rem) clamp(1.2rem,5vw,3.5rem);
  border-bottom:1px solid var(--line)}
.hero-grid{max-width:78rem;margin:0 auto;display:grid;gap:2.4rem;
  grid-template-columns:1.05fr .95fr;align-items:center}
.hero .eyebrow{color:#6f9be0;margin-bottom:1rem}
.hero h1{font-size:clamp(2.1rem,4.6vw,3.5rem);font-weight:800;
  letter-spacing:-.03em}
.lede{font-size:clamp(1rem,1.5vw,1.18rem);color:var(--hero-ink2);
  max-width:34em;margin:1.1rem 0 1.5rem}
.lede em{color:var(--hero-ink);font-style:normal;font-weight:600;
  border-bottom:2px solid var(--accent);padding-bottom:1px}
.hero-tags{display:flex;flex-wrap:wrap;gap:.6rem}
.pill{font-family:var(--mono);font-size:.76rem;font-weight:600;
  letter-spacing:.03em;padding:.42em .9em;border-radius:999px;
  display:inline-flex;align-items:center;gap:.5em;white-space:nowrap}
.pill.ok{background:var(--good);color:#fff}
.pill.no{background:var(--bad);color:#fff}
.pill.ghost{background:rgba(255,255,255,.07);color:var(--hero-ink2);
  border:1px solid rgba(255,255,255,.14)}
.stage{margin:0;background:rgba(255,255,255,.03);border:1px solid rgba(255,255,255,.1);
  border-radius:16px;padding:1rem 1rem .7rem;backdrop-filter:blur(2px)}
.stage canvas{width:100%;height:auto;display:block}
.stage figcaption{font-family:var(--mono);font-size:.72rem;color:var(--hero-ink2);
  margin-top:.6rem;display:flex;align-items:center;gap:.4em;justify-content:center;
  flex-wrap:wrap}
.dot{width:.62em;height:.62em;border-radius:50%;display:inline-block}
.dot.lead{background:#ffd24a}.dot.tail{background:var(--accent)}

/* ---- stat band ---- */
.tiles{max-width:78rem;margin:-1.5rem auto 0;padding:0 clamp(1.2rem,5vw,3.5rem);
  display:grid;grid-template-columns:repeat(6,1fr);gap:.8rem;position:relative;z-index:2}
.tile{background:var(--surface);border:1px solid var(--line);border-radius:13px;
  padding:1rem 1.05rem;box-shadow:0 6px 20px -12px rgba(10,20,40,.35)}
.tile .n{font-family:var(--mono);font-size:1.5rem;font-weight:700;
  letter-spacing:-.02em;line-height:1.1}
.tile .n .u{font-size:.72em;color:var(--muted);margin-left:.15em}
.tile .l{font-size:.74rem;color:var(--ink-2);margin-top:.35rem;line-height:1.3}
.tile.good .n{color:var(--good)}

/* ---- narrative blocks ---- */
main{max-width:78rem;margin:0 auto;padding:clamp(2rem,4vw,3.4rem) clamp(1.2rem,5vw,3.5rem)}
.block{display:grid;grid-template-columns:minmax(0,7fr) minmax(0,8fr);
  gap:clamp(1.4rem,3vw,2.8rem);padding:clamp(1.8rem,3.5vw,2.8rem) 0;
  align-items:start;border-top:1px solid var(--line)}
.block:first-child{border-top:none;padding-top:.5rem}
.block .kicker{color:var(--accent);margin-bottom:.7rem}
.block h2{font-size:clamp(1.4rem,2.6vw,2rem);font-weight:750;margin-bottom:.7rem}
.block h3{font-size:1.05rem;font-weight:700;margin:1.6rem 0 .5rem}
.lead-col{position:sticky;top:1.2rem}
.lead-col p{color:var(--ink-2);font-size:.96rem}
.lead-col p b{color:var(--ink);font-weight:650}
.fig-col{display:flex;flex-direction:column;gap:1.1rem;min-width:0}
.fine{font-size:.8rem;color:var(--muted);margin-top:.6rem}
@media (min-width:900px){.block.alt .lead-col{order:2}.block.alt .fig-col{order:1}}

/* ---- figure plates (kept light in both themes) ---- */
.plate{margin:0;background:var(--plate);border:1px solid var(--line);
  border-radius:12px;padding:.7rem .7rem .2rem;overflow:hidden}
.plate img{display:block;width:100%;height:auto;border-radius:5px}
.plate figcaption{font-size:.78rem;color:#4a5264;padding:.6rem .3rem .55rem;
  line-height:1.4;border-top:1px solid #eceef2;margin-top:.5rem}

/* ---- data tables ---- */
.data{width:100%;border-collapse:collapse;font-size:.85rem;margin:.4rem 0 .3rem}
.data th{text-align:left;font-family:var(--mono);font-size:.68rem;
  text-transform:uppercase;letter-spacing:.05em;color:var(--muted);
  font-weight:600;padding:.45em .7em;border-bottom:1px solid var(--line)}
.data td{padding:.5em .7em;border-bottom:1px solid var(--line);
  font-variant-numeric:tabular-nums}
.data td:not(:first-child){font-family:var(--mono);font-size:.9em}
.data tbody tr:last-child td{border-bottom:none}
.data tbody tr:hover{background:var(--surface-2)}
.ok{color:var(--good);font-weight:650}.bad{color:var(--bad);font-weight:650}

.pipe{display:flex;flex-wrap:wrap;align-items:center;gap:.5rem;margin:.3rem 0 1.2rem;
  font-family:var(--mono);font-size:.8rem}
.pipe span{background:var(--accent-soft);color:var(--accent);border-radius:8px;
  padding:.42em .8em;font-weight:600;border:1px solid color-mix(in srgb,var(--accent) 25%,transparent)}
.pipe .pipe-end{background:var(--good-soft);color:var(--good);
  border-color:color-mix(in srgb,var(--good) 30%,transparent)}
.pipe i{color:var(--muted);font-style:normal}
.verdict-line{margin-top:1.2rem}

/* ---- capabilities ---- */
.caps{max-width:78rem;margin:0 auto;padding:clamp(2rem,4vw,3rem) clamp(1.2rem,5vw,3.5rem);
  border-top:1px solid var(--line)}
.caps .kicker{color:var(--accent);margin-bottom:.6rem}
.caps h2{font-size:clamp(1.4rem,2.6vw,1.9rem);font-weight:750;margin-bottom:1.4rem}
.cap-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:1rem}
.cap{display:flex;gap:.85rem;background:var(--surface);border:1px solid var(--line);
  border-radius:13px;padding:1.1rem 1.15rem;align-items:flex-start}
.cap-mark{font-size:1.05rem;font-weight:800;width:1.9rem;height:1.9rem;flex:none;
  border-radius:8px;display:grid;place-items:center}
.cap.done .cap-mark{background:var(--good-soft);color:var(--good)}
.cap.partial .cap-mark{background:var(--accent-soft);color:var(--accent)}
.cap h4{font-size:.95rem;font-weight:700;margin-bottom:.2rem}
.cap p{font-size:.82rem;color:var(--ink-2);margin:0}

/* ---- footer ---- */
.foot{max-width:78rem;margin:0 auto;padding:2rem clamp(1.2rem,5vw,3.5rem) 3.5rem;
  border-top:1px solid var(--line);color:var(--ink-2);font-size:.85rem;
  display:flex;flex-direction:column;gap:.9rem}
.foot-cmd{display:flex;flex-wrap:wrap;gap:.5rem}
.foot-cmd code{background:var(--surface);border:1px solid var(--line);
  border-radius:7px;padding:.4em .7em;color:var(--ink-2)}
.foot .fine{margin:0}
.miss{color:var(--bad);font-family:var(--mono);font-size:.8rem}

@media (max-width:900px){
  .hero-grid{grid-template-columns:1fr}
  .tiles{grid-template-columns:repeat(3,1fr)}
  .block{grid-template-columns:1fr}
  .lead-col{position:static}
}
@media (max-width:560px){.tiles{grid-template-columns:repeat(2,1fr)}}
"""

CANVAS_JS = r"""
(function(){
  var cv=document.getElementById('platoon'); if(!cv) return;
  var ctx=cv.getContext('2d'), W=cv.width, H=cv.height;
  var reduce=window.matchMedia&&window.matchMedia('(prefers-reduced-motion:reduce)').matches;
  var N=7, r=0.72, A=26, baseY=H*0.56, x0=64, dx=(W-2*x0)/(N-1);
  var accent=(getComputedStyle(document.documentElement).getPropertyValue('--accent')||'#4f97ff').trim();
  function roundRect(x,y,w,h,rr){ctx.beginPath();ctx.moveTo(x+rr,y);
    ctx.arcTo(x+w,y,x+w,y+h,rr);ctx.arcTo(x+w,y+h,x,y+h,rr);
    ctx.arcTo(x,y+h,x,y,rr);ctx.arcTo(x,y,x+w,y,rr);ctx.closePath();}
  function veh(x,y,amp,lead){
    var w=34,h=15;
    ctx.save();
    if(lead){ctx.shadowColor='#ffd24a';ctx.shadowBlur=16;}
    ctx.fillStyle=lead?'#ffd24a':'rgba(120,160,220,'+(0.35+0.5*amp).toFixed(3)+')';
    ctx.strokeStyle=lead?'#ffe9a6':accent;
    ctx.lineWidth=1.4;
    roundRect(x-w/2,y-h/2,w,h,4); ctx.fill(); ctx.stroke();
    ctx.restore();
    ctx.fillStyle='rgba(10,16,28,.55)';
    ctx.beginPath();ctx.arc(x-9,y+h/2,2.4,0,7);ctx.arc(x+9,y+h/2,2.4,0,7);ctx.fill();
  }
  function frame(t){
    ctx.clearRect(0,0,W,H);
    ctx.strokeStyle='rgba(160,180,215,.16)';ctx.lineWidth=1;
    ctx.beginPath();ctx.moveTo(0,baseY+22);ctx.lineTo(W,baseY+22);ctx.stroke();
    ctx.setLineDash([10,12]);ctx.strokeStyle='rgba(160,180,215,.22)';
    ctx.beginPath();ctx.moveTo(0,baseY);ctx.lineTo(W,baseY);ctx.stroke();
    ctx.setLineDash([]);
    ctx.strokeStyle='rgba(120,160,220,.25)';ctx.lineWidth=1.5;ctx.beginPath();
    for(var i=0;i<N;i++){var yy=baseY-52-Math.pow(r,i)*20;
      i?ctx.lineTo(x0+i*dx,yy):ctx.moveTo(x0,yy);}
    ctx.stroke();
    var w=0.9;
    for(var i=0;i<N;i++){
      var amp=Math.pow(r,i);
      var off=A*amp*Math.sin(w*t - i*0.9);
      var x=x0+i*dx+off;
      ctx.strokeStyle='rgba(120,160,220,'+(0.25+0.55*amp).toFixed(3)+')';
      ctx.lineWidth=2;ctx.beginPath();
      ctx.moveTo(x0+i*dx,baseY-30);ctx.lineTo(x0+i*dx,baseY-30-Math.abs(off)*0.9);ctx.stroke();
      veh(x,baseY,amp,i===0);
    }
    ctx.fillStyle='rgba(210,225,250,.72)';
    ctx.font='11px ui-monospace,Consolas,monospace';ctx.textAlign='left';
    ctx.fillText('leader 100%',x0-18,baseY+40);
    ctx.textAlign='right';
    ctx.fillText('tail '+Math.round(Math.pow(r,N-1)*100)+'%',W-24,baseY+40);
  }
  if(reduce){frame(2.1);return;}
  var t0=performance.now();
  (function loop(now){frame((now-t0)/1000);requestAnimationFrame(loop);})(t0);
})();
"""
