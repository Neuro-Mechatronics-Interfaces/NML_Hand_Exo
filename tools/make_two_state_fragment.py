from __future__ import annotations

import json
import sys
from pathlib import Path


def main():
    data = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    payload = json.dumps(data, separators=(",", ":"))
    gesture = data["gesture"].split(":", 1)[-1].replace("_", " ")
    html = f'''<div id="two-state-plot" class="tsp-wrap">
  <div class="tsp-head"><div><div class="tsp-title">Rest ↔ {gesture}</div><div class="tsp-sub">Orientation-corrected log-RMS features · PCA reprojection · 12 sessions</div></div><label>Orientation <select id="tsp-orientation"><option value="all">All</option><option>neutral</option><option>pronated</option><option>supinated</option></select></label></div>
  <div class="tsp-metric">2-D training accuracy: <strong>{{metric}}</strong> · boundary shown in the same PCA axes</div>
  <canvas id="tsp-canvas" aria-label="Two-state PCA decision boundary"></canvas>
  <div class="tsp-legend"><span><i class="rest"></i>Rest</span><span><i class="gesture"></i>{gesture}</span><span class="line">— LDA boundary</span></div>
</div>
<script>(()=>{{const d={payload};const root=document.getElementById('two-state-plot');const c=document.getElementById('tsp-canvas');const ctx=c.getContext('2d');const metric=(100*d.training_balanced_accuracy_in_2d).toFixed(1)+'%';root.querySelector('.tsp-metric strong').textContent=metric;function draw(){{const ori=root.querySelector('select').value;const pts=d.points.filter(p=>ori==='all'||p.orientation===ori);const W=c.clientWidth||760,H=500,D=devicePixelRatio||1;c.width=W*D;c.height=H*D;ctx.setTransform(D,0,0,D,0,0);ctx.clearRect(0,0,W,H);const pad={{l:54,r:24,t:20,b:46}},xmin=d.xlim[0],xmax=d.xlim[1],ymin=d.ylim[0],ymax=d.ylim[1];const sx=x=>pad.l+(x-xmin)/(xmax-xmin||1)*(W-pad.l-pad.r),sy=y=>H-pad.b-(y-ymin)/(ymax-ymin||1)*(H-pad.t-pad.b);ctx.strokeStyle='#3b3b3b';ctx.beginPath();ctx.moveTo(pad.l,pad.t);ctx.lineTo(pad.l,H-pad.b);ctx.lineTo(W-pad.r,H-pad.b);ctx.stroke();ctx.fillStyle='#9aa0a6';ctx.font='12px system-ui';ctx.fillText('PCA 1',W/2-18,H-12);ctx.save();ctx.translate(14,H/2+18);ctx.rotate(-Math.PI/2);ctx.fillText('PCA 2',0,0);ctx.restore();for(const p of pts){{ctx.globalAlpha=.52;ctx.fillStyle=p.label==='rest'?'#61a5ff':'#ff9f43';ctx.beginPath();ctx.arc(sx(p.x),sy(p.y),2.7,0,Math.PI*2);ctx.fill();}}ctx.globalAlpha=1;ctx.strokeStyle='#f3f4f6';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(sx(d.boundary[0].x),sy(d.boundary[0].y));ctx.lineTo(sx(d.boundary[1].x),sy(d.boundary[1].y));ctx.stroke();}}root.querySelector('select').addEventListener('change',draw);new ResizeObserver(draw).observe(c);draw();}})();</script>
<style>#two-state-plot{{font:14px system-ui,sans-serif;color:#e9e9e9}}.tsp-head{{display:flex;justify-content:space-between;align-items:end;gap:14px;margin-bottom:8px}}.tsp-title{{font-size:18px;font-weight:600}}.tsp-sub,.tsp-metric{{font-size:12px;color:#9aa0a6}}.tsp-metric{{margin-bottom:8px}}.tsp-metric strong{{color:#ffb86b}}select{{background:#222;color:#eee;border:1px solid #555;border-radius:4px;padding:4px 7px;margin-left:5px}}#tsp-canvas{{display:block;width:100%;height:500px;background:#171717;border:1px solid #343434;border-radius:6px}}.tsp-legend{{display:flex;gap:16px;margin-top:8px;color:#bfc3c8;font-size:12px}}.tsp-legend i{{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:5px}}.tsp-legend .rest{{background:#61a5ff}}.tsp-legend .gesture{{background:#ff9f43}}.tsp-legend .line{{color:#eee}}</style>'''
    Path(sys.argv[2]).write_text(html, encoding="utf-8")


if __name__ == "__main__":
    main()
