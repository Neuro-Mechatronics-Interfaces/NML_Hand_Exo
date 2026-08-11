#!/usr/bin/env python
"""Generate conservative Aim 3 proposal figures from one recording folder."""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle

try:
    import yaml
except Exception:
    yaml = None

FILES = {
    "control": "control_commands.csv",
    "communication": "communication_log.csv",
    "telemetry": "motor_telemetry.csv",
    "events": "event_markers.csv",
    "decoder": "decoder_output.csv",
    "features": "processed_features.csv",
    "raw_emg": "raw_emg.csv",
}
MOTOR_LABELS = {16: "index", 17: "middle", 18: "ring"}
DEFAULT_COLUMNS = {
    "time": ["time_s", "timestamp_s", "timestamp", "timestamp_lsl_user_intent", "timestamp_control_update_monotonic_ns", "timestamp_control_update_wall_ns", "timestamp_script_send_monotonic_ns", "timestamp_receive_monotonic_ns"],
    "effort": ["signed_effort_u_deadbanded", "signed_effort_u_clipped", "mapped_intent", "smoothed_intent", "raw_intent", "User_Intent", "user_intent"],
    "current": ["commanded_current_mA", "command_current_ma", "I_cmd_mA"],
    "motor": ["motor_id", "dxl_id"],
    "position": ["position", "present_position_ticks", "position_ticks"],
    "measured_current": ["measured_current_mA", "motor_current_ma", "present_current_mA"],
}

@dataclass
class Artifact:
    name: str
    status: str
    reason: str = ""
    files: list[str] = field(default_factory=list)

@dataclass
class Ctx:
    data_dir: Path
    out_dir: Path
    cfg: dict[str, Any]
    frames: dict[str, pd.DataFrame]
    metadata: dict[str, Any]
    warnings: list[str] = field(default_factory=list)
    artifacts: list[Artifact] = field(default_factory=list)
    captions: list[tuple[str, str]] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)

    @property
    def figures(self) -> Path: return self.out_dir / "figures"
    @property
    def tables(self) -> Path: return self.out_dir / "tables"
    @property
    def reports(self) -> Path: return self.out_dir / "reports"
    def record(self, name: str, status: str, reason: str = "", files: list[Path] | None = None) -> None:
        self.artifacts.append(Artifact(name, status, reason, [str(p) for p in files or []]))
    def caption(self, name: str, text: str) -> None:
        self.captions.append((name, text))

def load_config(path: Path | None) -> dict[str, Any]:
    cfg = {"figure": {"dpi": 300, "width_in": 13.333, "height_in": 7.5, "font_size": 14}, "control": {"default_deadband": 0.05, "preliminary_i_max_ma": 200.0, "current_limit_ma": 910.0, "active_motor_ids": [16, 17, 18]}, "columns": DEFAULT_COLUMNS}
    if path and path.exists() and yaml is not None:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        for key, val in loaded.items():
            if isinstance(val, dict) and isinstance(cfg.get(key), dict): cfg[key].update(val)
            else: cfg[key] = val
    return cfg

def load_session(data_dir: Path):
    frames, warnings = {}, []
    metadata = {}
    mp = data_dir / "metadata.json"
    if mp.exists():
        try: metadata = json.loads(mp.read_text(encoding="utf-8"))
        except Exception as exc: warnings.append(f"metadata.json could not be read: {exc}")
    for key, name in FILES.items():
        p = data_dir / name
        if not p.exists(): continue
        try: frames[key] = pd.read_csv(p)
        except pd.errors.EmptyDataError: frames[key] = pd.DataFrame()
        except Exception as exc: warnings.append(f"{name} could not be read: {exc}")
    return frames, metadata, warnings

def col(df: pd.DataFrame, names: list[str]) -> str | None:
    lower = {c.lower(): c for c in df.columns}
    for n in names:
        if n in df.columns: return n
        if n.lower() in lower: return lower[n.lower()]
    return None

def num(df: pd.DataFrame, c: str) -> pd.Series:
    return pd.to_numeric(df[c], errors="coerce")

def t_axis(df: pd.DataFrame, ctx: Ctx):
    c = col(df, ctx.cfg["columns"].get("time", DEFAULT_COLUMNS["time"]))
    if c is None: return pd.Series(np.arange(len(df), dtype=float), index=df.index), "sample"
    t = num(df, c)
    if c.endswith("_ns"): t = t / 1e9
    if t.notna().any(): t = t - t.dropna().iloc[0]
    return t, c

def setup(ctx: Ctx):
    for d in [ctx.out_dir, ctx.figures, ctx.tables, ctx.reports]: d.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": ctx.cfg["figure"]["font_size"]})

def save(ctx: Ctx, fig, stem: str):
    png, svg = ctx.figures / f"{stem}.png", ctx.figures / f"{stem}.svg"
    fig.tight_layout(); fig.savefig(png, dpi=int(ctx.cfg["figure"]["dpi"]), bbox_inches="tight"); fig.savefig(svg, bbox_inches="tight"); plt.close(fig)
    return [png, svg]

def fig1(ctx: Ctx):
    fig, ax = plt.subplots(figsize=(ctx.cfg["figure"]["width_in"], ctx.cfg["figure"]["height_in"]))
    ax.axis("off")
    labels = ["8-channel\nforearm EMG", "RMS feature\nextraction", "LDA effort-axis\nprojection", "Rest deadband +\nnormalization", "Signed effort\nu(t) in [-1, 1]", "I_cmd = G *\nI_max * u(t)", "Index / middle / ring\ncurrent command", "Exoskeleton\nfinger motion"]
    x0, y, w, h = 0.03, 0.46, 0.105, 0.18; gap = (0.94 - len(labels) * w) / (len(labels) - 1)
    for i, label in enumerate(labels):
        x = x0 + i * (w + gap); ax.add_patch(Rectangle((x, y), w, h, transform=ax.transAxes, facecolor="#f5f7fb", edgecolor="#264653", linewidth=1.8)); ax.text(x + w/2, y + h/2, label, ha="center", va="center", transform=ax.transAxes, fontsize=11)
        if i < len(labels)-1: ax.add_patch(FancyArrowPatch((x+w, y+h/2), (x+w+gap*0.86, y+h/2), transform=ax.transAxes, arrowstyle="->", mutation_scale=18, linewidth=1.5, color="#264653"))
    ax.set_title("Aim 3 preliminary proportional EMG-to-exoskeleton control pipeline", weight="bold")
    files = save(ctx, fig, "fig1_pipeline_emg_to_exo")
    ctx.caption("fig1_pipeline_emg_to_exo", "Preliminary proportional control pipeline converting 8-channel forearm EMG into signed bidirectional motor current commands for hand exoskeleton assistance.")
    ctx.record("Figure 1: System architecture diagram", "generated", files=files)

def effort_source(ctx: Ctx):
    for key in ["decoder", "control"]:
        df = ctx.frames.get(key)
        if df is not None and not df.empty:
            c = col(df, ctx.cfg["columns"].get("effort", DEFAULT_COLUMNS["effort"]))
            if c: return df, key, c
    return None, "", None

def fig3(ctx: Ctx):
    df, key, effort_col = effort_source(ctx)
    if df is None or effort_col is None:
        ctx.record("Figure 3: Signed effort time series", "skipped", "decoder_output.csv or control_commands.csv with an effort column is required."); return
    t, label = t_axis(df, ctx); effort = num(df, effort_col)
    fig, ax = plt.subplots(figsize=(ctx.cfg["figure"]["width_in"], ctx.cfg["figure"]["height_in"]))
    ax.plot(t, effort, color="#264653", linewidth=2, label=effort_col)
    db = float(ctx.metadata.get("deadband_threshold", ctx.cfg["control"].get("default_deadband", 0.05)))
    ax.axhspan(-db, db, color="#f1c40f", alpha=0.14, label="deadband")
    ax.axhline(0, color="#999999", linewidth=0.8)
    ax.set_xlabel(f"Time from start (s, from {label})"); ax.set_ylabel("Signed effort command"); ax.set_title("Deadbanded signed effort command over time"); ax.legend(loc="best")
    vals = effort.dropna(); tx = t.dropna().drop_duplicates()
    if len(vals):
        ctx.metrics["effort_peak_positive"] = float(vals.max()); ctx.metrics["effort_peak_negative"] = float(vals.min()); ctx.metrics["effort_saturation_fraction"] = float((vals.abs() >= 0.999).mean())
    if len(tx) > 1 and float(tx.iloc[-1] - tx.iloc[0]) > 0: ctx.metrics["decoder_or_control_update_rate_hz"] = float((len(tx)-1)/(tx.iloc[-1]-tx.iloc[0]))
    files = save(ctx, fig, "fig3_signed_effort_timeseries")
    ctx.caption("fig3_signed_effort_timeseries", "Deadbanding converts the LDA projection into a stable signed effort command for bidirectional hand exoskeleton control.")
    ctx.record("Figure 3: Signed effort time series", "generated", f"Generated from {FILES[key]} using {effort_col}.", files)

def command_pivot(ctx: Ctx):
    df = ctx.frames.get("control")
    if df is None or df.empty: return None, None, "control_commands.csv is missing or empty."
    motor = col(df, ctx.cfg["columns"].get("motor", DEFAULT_COLUMNS["motor"])); curr = col(df, ctx.cfg["columns"].get("current_command", ctx.cfg["columns"].get("current", DEFAULT_COLUMNS["current"])))
    if not motor or not curr: return None, None, "control_commands.csv needs motor_id and commanded current columns."
    t, _ = t_axis(df, ctx); work = df.copy(); work["_t"] = t; work["_motor"] = pd.to_numeric(work[motor], errors="coerce").astype("Int64"); work["_current"] = num(work, curr)
    pivot = work.pivot_table(index="_t", columns="_motor", values="_current", aggfunc="last").sort_index()
    e_col = col(df, ctx.cfg["columns"].get("effort", DEFAULT_COLUMNS["effort"])); effort = pd.to_numeric(work.groupby("_t")[e_col].first(), errors="coerce") if e_col else None
    return pivot, effort, ""

def fig4(ctx: Ctx):
    pivot, effort, reason = command_pivot(ctx)
    if pivot is None: ctx.record("Figure 4: Effort to current command", "skipped", reason); return
    tele = ctx.frames.get("telemetry"); rows = 3 if tele is not None and not tele.empty else 2
    fig, axes = plt.subplots(rows, 1, figsize=(ctx.cfg["figure"]["width_in"], ctx.cfg["figure"]["height_in"]), squeeze=False); axes = [axes[i,0] for i in range(rows)]
    if effort is not None and effort.notna().any(): axes[0].plot(effort.index, effort.values, color="#264653", linewidth=2)
    else: axes[0].text(0.5, 0.5, "No effort column found", ha="center", va="center", transform=axes[0].transAxes)
    axes[0].axhline(0, color="#999", linewidth=0.8); axes[0].set_ylabel("u(t)"); axes[0].set_title("Signed EMG effort and exoskeleton current commands")
    for mid in pivot.columns:
        axes[1].plot(pivot.index, pivot[mid], linewidth=1.8, label=MOTOR_LABELS.get(int(mid), f"ID {mid}"))
    axes[1].axhline(0, color="#999", linewidth=0.8); axes[1].set_ylabel("Current command (mA)"); axes[1].legend(loc="best", ncols=3)
    if rows == 3:
        motor = col(tele, ctx.cfg["columns"].get("motor", DEFAULT_COLUMNS["motor"])); pos = col(tele, ctx.cfg["columns"].get("position", DEFAULT_COLUMNS["position"])); cur = col(tele, ctx.cfg["columns"].get("measured_current", DEFAULT_COLUMNS["measured_current"])); ycol = pos or cur
        if motor and ycol:
            tt, _ = t_axis(tele, ctx); tw = tele.copy(); tw["_t"] = tt; tw["_motor"] = pd.to_numeric(tw[motor], errors="coerce").astype("Int64"); tw["_y"] = num(tw, ycol); tp = tw.pivot_table(index="_t", columns="_motor", values="_y", aggfunc="last").sort_index()
            for mid in tp.columns: axes[2].plot(tp.index, tp[mid], linewidth=1.5, label=MOTOR_LABELS.get(int(mid), f"ID {mid}"))
            axes[2].set_ylabel(ycol); axes[2].legend(loc="best", ncols=3)
        else: axes[2].text(0.5, 0.5, "Telemetry present but no motor_id + position/current columns detected", ha="center", va="center", transform=axes[2].transAxes)
    axes[-1].set_xlabel("Time from start (s)")
    abs_curr = pivot.abs(); ctx.metrics["peak_abs_commanded_current_mA"] = float(abs_curr.max().max()); ctx.metrics["mean_abs_commanded_current_mA"] = float(abs_curr.stack().mean())
    imax = float(ctx.metadata.get("I_max_mA", ctx.cfg["control"].get("preliminary_i_max_ma", 200.0))); ctx.metrics["command_saturation_fraction"] = float((abs_curr >= imax*0.999).stack().mean()) if imax else ""
    files = save(ctx, fig, "fig4_effort_to_current_command")
    ctx.caption("fig4_effort_to_current_command", "Decoded EMG effort is converted into graded bidirectional current commands for index, middle, and ring exoskeleton motors.")
    ctx.record("Figure 4: Effort to current command", "generated", files=files)

def mark_skips(ctx: Ctx):
    if "features" not in ctx.frames and "raw_emg" not in ctx.frames: ctx.record("Figure 2: LDA signed effort axis", "skipped", "processed_features.csv or raw_emg.csv with open/rest/close labels is required.")
    control = ctx.frames.get("control")
    if control is None or control.empty or "gain_G" not in control.columns or pd.to_numeric(control["gain_G"], errors="coerce").dropna().nunique() < 2: ctx.record("Figure 5: Gain sweep", "skipped", "At least two gain_G values are required.")
    tele = ctx.frames.get("telemetry")
    if tele is None or tele.empty: ctx.record("Figure 6: Motor response / latency estimate", "skipped", "motor_telemetry.csv has no rows, so motor response latency cannot be estimated.")
    videos = list(ctx.data_dir.glob("*.mp4")) + list((ctx.data_dir / "video").glob("*.mp4") if (ctx.data_dir / "video").exists() else [])
    if not videos: ctx.record("Figure 7: Video storyboard", "skipped", "No video files found in session folder or video/ subfolder.")

def table(ctx: Ctx):
    control, comm, tele, md = ctx.frames.get("control"), ctx.frames.get("communication"), ctx.frames.get("telemetry"), ctx.metadata
    row: dict[str, Any] = {"session_id": md.get("session_id", ctx.data_dir.name), "participant_label": md.get("participant_id_or_self_demo_label", ""), "number_of_emg_channels": md.get("emg_channel_count", ""), "feature_window_ms": md.get("feature_window_ms", ""), "I_max_mA": md.get("I_max_mA", ""), "active_motors": ",".join(map(str, md.get("motor_ids_active", ctx.cfg["control"].get("active_motor_ids", [16,17,18])))), "motor_telemetry_rows": 0 if tele is None else len(tele), "latency_estimate_s": ctx.metrics.get("command_to_first_telemetry_latency_s", "not measured")}
    if control is not None and not control.empty:
        if "gain_G" in control.columns: row["gain_values_tested"] = ",".join(map(str, sorted(pd.to_numeric(control["gain_G"], errors="coerce").dropna().unique())))
        t, _ = t_axis(control, ctx); ticks = t.dropna().drop_duplicates(); dur = float(ticks.iloc[-1] - ticks.iloc[0]) if len(ticks) > 1 else 0
        if dur > 0: row["control_update_rate_hz"] = (len(ticks)-1)/dur
        if "safety_limited_boolean" in control.columns: row["number_of_safety_limited_events"] = int(control["safety_limited_boolean"].astype(str).str.lower().eq("true").sum())
        if "stale_lsl" in control.columns: row["stale_lsl_rows"] = int(control["stale_lsl"].astype(str).str.lower().eq("true").sum())
    if comm is not None and not comm.empty:
        errs = 0
        for c in ["error", "communication_error_code"]:
            if c in comm.columns: errs += int(comm[c].astype(str).str.strip().replace("nan", "").ne("").sum())
        row["communication_error_count"] = errs
    row.update(ctx.metrics); row["missing_data_notes"] = " | ".join(a.reason for a in ctx.artifacts if a.status == "skipped" and a.reason)
    df = pd.DataFrame([row]); csv = ctx.tables / "table1_aim3_demo_metrics.csv"; mdp = ctx.tables / "table1_aim3_demo_metrics.md"; df.to_csv(csv, index=False)
    headers = list(df.columns); values = ["" if pd.isna(v) else str(v) for v in df.iloc[0].tolist()]
    mdp.write_text("| " + " | ".join(headers) + " |\n| " + " | ".join(["---"] * len(headers)) + " |\n| " + " | ".join(values) + " |\n", encoding="utf-8")
    ctx.record("Table 1: Aim 3 demo metrics", "generated", files=[csv, mdp])

def reports(ctx: Ctx):
    (ctx.reports / "captions.md").write_text("# Captions\n\n" + "\n\n".join(f"## {n}\n{text}" for n, text in ctx.captions) + "\n", encoding="utf-8")
    gen = [a for a in ctx.artifacts if a.status == "generated"]; skip = [a for a in ctx.artifacts if a.status == "skipped"]; avail = sorted(FILES[k] for k in ctx.frames); missing = sorted(v for k, v in FILES.items() if k not in ctx.frames)
    lines = ["# Aim 3 Figure Readiness Report", "", f"Data directory: `{ctx.data_dir}`", f"Session ID: `{ctx.metadata.get('session_id', ctx.data_dir.name)}`", "", "## Generated"] + ([f"- {a.name}" for a in gen] or ["- None"])
    lines += ["", "## Skipped"] + ([f"- {a.name}: {a.reason}" for a in skip] or ["- None"])
    lines += ["", "## Available Data Files"] + ([f"- {x}" for x in avail] or ["- None"])
    lines += ["", "## Missing Data Files"] + ([f"- {x}" for x in missing] or ["- None"])
    lines += ["", "## Reliable Claims From These Data", "- Preliminary able-bodied engineering feasibility demonstration.", "- Signed User_Intent can be converted into bidirectional current commands when control_commands.csv is present.", "- Command update rate, packet continuity, current-command magnitude, and send errors can be quantified.", "", "## Claims To Avoid Unless Additional Data Are Present", "- SCI feasibility or clinical validation.", "- Assistance reduces user effort unless EMG reduction is measured under assistance conditions.", "- Functional improvement unless task performance is measured.", "- Motor torque unless current-to-torque calibration is performed.", "- Latency below any threshold unless telemetry and timing logs support it.", "", "## Recommended Next Recording Actions", "- Confirm GUI UDP telemetry status shows sent-frame counts during recording.", "- Record or link decoder_output.csv and processed_features.csv with open/rest/close labels.", "- Record motor telemetry rows with host receive timestamps and firmware timestamp_ms when fast telemetry is active.", "- Use a consistent I_max_mA in metadata and captions."]
    if ctx.warnings: lines += ["", "## Warnings"] + [f"- {w}" for w in ctx.warnings]
    ready = ctx.reports / "aim3_figure_readiness_report.md"; ready.write_text("\n".join(lines) + "\n", encoding="utf-8")
    miss = ctx.reports / "missing_data_report.md"; miss.write_text("# Missing Data Report\n\n" + ("\n".join(f"- {a.name}: {a.reason}" for a in skip) if skip else "No skipped figures from missing data.") + "\n", encoding="utf-8"); ctx.record("Reports and captions", "generated", files=[ready, miss, ctx.reports / "captions.md"])

def run(ctx: Ctx):
    setup(ctx); fig1(ctx); fig3(ctx); fig4(ctx); mark_skips(ctx); table(ctx); reports(ctx)

def main() -> int:
    p = argparse.ArgumentParser(description="Generate Aim 3 proposal figures from a recording session."); p.add_argument("--data_dir", required=True, type=Path); p.add_argument("--out_dir", default=Path("aim3_analysis/outputs"), type=Path); p.add_argument("--config", default=Path("aim3_analysis/aim3_config.yaml"), type=Path); a = p.parse_args()
    if not a.data_dir.exists(): raise SystemExit(f"data_dir does not exist: {a.data_dir}")
    frames, metadata, warnings = load_session(a.data_dir); ctx = Ctx(a.data_dir.resolve(), a.out_dir.resolve(), load_config(a.config), frames, metadata, warnings); run(ctx)
    print(f"Generated Aim 3 analysis outputs in: {ctx.out_dir}"); print(f"Artifacts generated: {sum(x.status == 'generated' for x in ctx.artifacts)}; skipped: {sum(x.status == 'skipped' for x in ctx.artifacts)}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
