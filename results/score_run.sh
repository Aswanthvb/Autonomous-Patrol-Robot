#!/usr/bin/env bash
# usage: score_run.sh <detection_run_dir>
# Scores one run with the strict analyzer (v4: centre-point visibility) and the lenient one (v5: any-part
# visibility, fast turns not scored). Results go to <run>/analysis_v4 and <run>/analysis_v5.
D="${1%/}"; T=~/patrol_ws/src/patrol_perception/tools
[ -f "$D/frames.csv" ] || { echo "Not a detection run folder: $D"; exit 1; }
[ -f "$T/analyze_detections.py.v4_reconstructed.bak" ] || { echo "Missing $T/analyze_detections.py.v4_reconstructed.bak"; exit 1; }
for v in v4 v5; do
  if [ "$v" = v4 ]; then S="$T/analyze_detections.py.v4_reconstructed.bak"; else S="$T/analyze_detections.py"; fi
  mkdir -p "$D/analysis_$v"
  echo "=============== analyzer $v ==============="
  python3 "$S" "$D" | tee "$D/analysis_$v/printout.txt"
  mv "$D"/detection_by_distance.csv "$D"/detection_passes.csv "$D"/false_positives.csv "$D"/detection_summary.yaml "$D/analysis_$v/" 2>/dev/null
done
echo; echo "Saved in $D/analysis_v4 and $D/analysis_v5"
