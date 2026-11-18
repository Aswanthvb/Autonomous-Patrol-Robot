#!/usr/bin/env bash
label="$1"; [ -n "$label" ] || { echo "usage: rename_run.sh <label>"; exit 1; }
P=$(find ~/patrol_runs -maxdepth 1 -type d -name 'run_*' -mmin -90 -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
D=$(find ~/detection_runs -maxdepth 1 -type d -name 'det_*' -mmin -90 -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
if [ -z "$P" ] || [ -z "$D" ]; then
  echo "No run folders from the last 90 minutes (patrol: '$P', detection: '$D'). Nothing renamed."; exit 1
fi
echo "Patrol run   : $P"; cat "$P/run_summary.yaml"
echo "Detection run: $D"; cat "$D/logger_info.yaml" 2>/dev/null
read -p "Rename to '$label'? [y/N] " a
[ "$a" = y ] || { echo cancelled; exit 0; }
mv "$P" ~/patrol_runs/"${label}_patrol" && mv "$D" ~/detection_runs/"$label" && echo done
