# Aim 3 Analysis

Generate thesis-proposal-ready figures and reports from Aim 3 EMG-to-hand-exoskeleton recordings.

Run from the repository root:

```powershell
python aim3_analysis\generate_aim3_figures.py --data_dir data\Aim3_EMG_Exo_Demo\2026-07-07_Aim3_20260707_134347 --out_dir aim3_analysis\outputs
```

The script inspects the available files and only generates figures supported by the data. Missing inputs are documented in `reports/aim3_figure_readiness_report.md` and `reports/missing_data_report.md`.

Expected session files can include:

- `metadata.json`
- `control_commands.csv`
- `communication_log.csv`
- `motor_telemetry.csv`
- `event_markers.csv`
- `decoder_output.csv`
- `processed_features.csv`
- `raw_emg.csv`
- video or screen recordings

Interpretation guardrails:

- Treat this as a preliminary able-bodied engineering feasibility demonstration.
- Do not claim SCI feasibility, clinical benefit, reduced user effort, or functional improvement from this demo alone.
- Do not call current command torque unless torque has been calibrated.
- Do not claim formal latency unless command and telemetry timestamps are present.
