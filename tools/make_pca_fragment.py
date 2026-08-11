from __future__ import annotations

import json
import sys
from pathlib import Path


def main():
    data = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    payload = json.dumps(data, separators=(",", ":"))
    fragment = f'''<div id="jonathan-emg-pca" class="jep-wrap">
  <div class="jep-head">
    <div>
      <div class="jep-title">EMG gesture separability · LDA</div>
      <div class="jep-subtitle">Jonathan · 12 sessions · neutral / pronated / supinated</div>
    </div>
    <label class="jep-filter">Wrist orientation
      <select id="jep-orientation"><option value="all">All</option><option value="neutral">Neutral</option><option value="pronated">Pronated</option><option value="supinated">Supinated</option></select>
    </label>
  </div>
  <div class="jep-metric">Gesture-only LDA silhouette score: <strong>{data["lda_silhouette_gestures_only"]:.3f}</strong> · LDA uses the known gesture labels to maximize between-class separation</div>
  <canvas id="jep-canvas" role="img" aria-label="Two-dimensional LDA scatter plot of EMG gesture windows"></canvas>
  <div class="jep-legend" id="jep-legend"></div>
</div>
<script>
(() => {{
  const root = document.getElementById('jonathan-emg-pca');
  const data = {payload};
  const canvas = document.getElementById('jep-canvas');
  const ctx = canvas.getContext('2d');
  const colors = ['#54a0ff','#ff9f43','#1dd1a1','#ee5253','#a29bfe','#feca57','#48dbfb','#ff6b6b','#10ac84','#5f27cd','#c8d6e5','#8395a7','#222f3e','#ff9ff3','#00d2d3','#576574','#341f97'];
  const gestures = [...new Set(data.points.map(p => p.gesture))].sort();
  const color = Object.fromEntries(gestures.map((g,i) => [g, colors[i % colors.length]]));
  document.getElementById('jep-legend').innerHTML = gestures.map(g => `<span><i style="background:${{color[g]}}"></i>${{g}}</span>`).join('');
  function draw() {{
    const orientation = document.getElementById('jep-orientation').value;
    const points = data.points.filter(p => orientation === 'all' || p.orientation === orientation);
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth || 736, h = 500;
    canvas.width = w * dpr; canvas.height = h * dpr; ctx.setTransform(dpr,0,0,dpr,0,0);
    ctx.clearRect(0,0,w,h);
    const pad = {{l:52,r:18,t:18,b:42}};
    const xs = points.map(p => p.lda1), ys = points.map(p => p.lda2);
    const xmin = Math.min(...xs), xmax = Math.max(...xs), ymin = Math.min(...ys), ymax = Math.max(...ys);
    const sx = x => pad.l + (x-xmin)/(xmax-xmin || 1)*(w-pad.l-pad.r);
    const sy = y => h-pad.b - (y-ymin)/(ymax-ymin || 1)*(h-pad.t-pad.b);
    ctx.strokeStyle='#3a3a3a'; ctx.lineWidth=1; ctx.beginPath(); ctx.moveTo(pad.l,pad.t); ctx.lineTo(pad.l,h-pad.b); ctx.lineTo(w-pad.r,h-pad.b); ctx.stroke();
    ctx.fillStyle='#aaa'; ctx.font='12px sans-serif'; ctx.fillText('PC1', w/2-10, h-10); ctx.save(); ctx.translate(14,h/2+18); ctx.rotate(-Math.PI/2); ctx.fillText('PC2',0,0); ctx.restore();
    for (const p of points) {{ ctx.globalAlpha = p.gesture === 'rest' ? 0.18 : 0.55; ctx.fillStyle=color[p.gesture]; ctx.beginPath(); ctx.arc(sx(p.pc1),sy(p.pc2),2.4,0,Math.PI*2); ctx.fill(); }}
    ctx.globalAlpha=1;
  }}
  document.getElementById('jep-orientation').addEventListener('change', draw);
  new ResizeObserver(draw).observe(canvas);
  draw();
}})();
</script>
<style>
#jonathan-emg-pca{{font:14px system-ui,sans-serif;color:#e9e9e9;max-width:100%;}}
.jep-head{{display:flex;justify-content:space-between;align-items:end;gap:16px;margin-bottom:8px;}}
.jep-title{{font-size:18px;font-weight:500;}} .jep-subtitle,.jep-metric{{color:#9aa0a6;font-size:12px;}}
.jep-filter{{font-size:12px;color:#bfc3c8;}} .jep-filter select{{margin-left:6px;background:#222;color:#eee;border:1px solid #555;border-radius:4px;padding:4px 7px;}}
.jep-metric{{margin-bottom:8px;}} .jep-metric strong{{color:#ffb86b;}}
#jep-canvas{{display:block;width:100%;height:500px;background:#171717;border:1px solid #343434;border-radius:6px;}}
.jep-legend{{display:flex;flex-wrap:wrap;gap:8px 14px;margin-top:8px;color:#bfc3c8;font-size:11px;}} .jep-legend span{{display:inline-flex;align-items:center;gap:4px;}} .jep-legend i{{width:8px;height:8px;border-radius:50%;display:inline-block;}}
</style>'''
    Path(sys.argv[2]).write_text(fragment, encoding="utf-8")


if __name__ == "__main__":
    main()
