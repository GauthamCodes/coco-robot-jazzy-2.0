#!/usr/bin/env bash
# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# resume_curriculum.sh — restart an interrupted curriculum run after a reboot.
#
# train_curriculum.sh installs this as a GNOME login hook
# (~/.config/autostart/coco-curriculum-resume.desktop) so that a shutdown,
# power loss or kernel panic costs at most the steps since the last
# checkpoint instead of the whole run. It is deliberately conservative and
# does nothing unless every condition below holds:
#
#   * the newest run directory has an AUTORESUME marker (the run asked for it)
#   * that run has no DONE marker (it really was interrupted)
#   * no curriculum is already running (so a login during training is a no-op)
#
# Otherwise it removes the login hook and exits, so it can never turn into a
# process that starts heavy training on every login forever.
#
# Run it by hand any time with:  ./resume_curriculum.sh
# Disable the hook with:         rm ~/.config/autostart/coco-curriculum-resume.desktop
set -o pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNS_ROOT="$HOME/coco_rl_runs"
HOOK="$HOME/.config/autostart/coco-curriculum-resume.desktop"
LOG="$RUNS_ROOT/resume.log"

say() { echo "[$(date -Is)] $*" >>"$LOG"; echo "$*"; }

mkdir -p "$RUNS_ROOT"

RUN="$(ls -d "$RUNS_ROOT"/curriculum_* 2>/dev/null | tail -1)"
if [ -z "$RUN" ]; then
  say "no run directory under $RUNS_ROOT — nothing to resume; removing hook"
  rm -f "$HOOK"
  exit 0
fi

if [ ! -f "$RUN/AUTORESUME" ]; then
  say "$RUN did not opt into auto-resume — removing hook"
  rm -f "$HOOK"
  exit 0
fi

if [ -f "$RUN/DONE" ]; then
  say "$RUN already finished — removing hook"
  rm -f "$HOOK"
  exit 0
fi

# Already training? Then this is an ordinary login, not a recovery.
if pgrep -f 'bash .*train_curriculum.sh' >/dev/null 2>&1; then
  say "a curriculum is already running — nothing to do"
  exit 0
fi

say "resuming interrupted run $RUN"
cd "$REPO" || exit 1
# Detached exactly like a fresh launch: own session, no controlling terminal,
# so it survives the login shell that started it going away.
setsid nohup ./train_curriculum.sh --resume-run "$RUN" \
    >/dev/null 2>&1 </dev/null &
say "relaunched (watch with ./watch_training.py)"
