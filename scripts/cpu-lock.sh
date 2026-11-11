#!/bin/sh
# usage: scripts/cpu-lock.sh WHAT COMMAND... -> runs COMMAND holding /tmp/claude-<uid>/cpu-heavy.lock, shared with other agent
# sessions, for timing-dependent jobs only (docs/agents/agent-orchestration.md). A hold ends by itself after 30 min
# (timeout). A waiter goes ahead at once when the holder's agent session is gone, and once the current hold is 30 min old
# in any case (the owner line's start; the waiter's own wait when the line is empty or unreadable): it stops whatever still
# holds the lock (inherited descriptors included) once, then queues on the lock itself, so a job that took the lock
# meanwhile runs to its end. Ported from gran-turismo-decomp tools/cpu_lock.sh (tested there).
D=${CPU_LOCK_DIR:-/tmp/claude-$(id -u)}; LOCK=$D/cpu-heavy.lock; OWNER=$D/cpu-heavy.owner
STARVE=${CPU_LOCK_STARVE_S:-1800}; POLL=${CPU_LOCK_POLL_S:-30}; GRACE=${CPU_LOCK_GRACE_S:-15}  # overridable for tests
what=$1; shift
session_pid() {  # the agent session (the nearest `claude` ancestor), or the parent shell if there is none
  p=$$; while [ "$p" -gt 1 ]; do [ "$(cat /proc/$p/comm 2>/dev/null)" = claude ] && { echo $p; return; }
    p=$(awk '{print $4}' /proc/$p/stat 2>/dev/null || echo 1); done; echo $PPID
}
holders() {  # processes with the lock file open that are not waiting for it (waiters: flock, this script)
  for p in $(fuser "$LOCK" 2>/dev/null); do
    [ "$p" = $$ ] && continue
    case "$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)" in flock\ *|*cpu_lock*|*cpu-lock*) continue;; esac
    echo "$p"
  done
}
exec 9>"$LOCK"
waited=0; stopped="(none)"; starved=false  # not "": an empty owner line (a job ended, a child kept the descriptor) is stopped too
until flock -n 9; do
  owner=$(cat "$OWNER" 2>/dev/null); spid=$(echo "$owner" | cut -d' ' -f3)
  gone=false
  # the limit counts the hold's age, not this waiter's wait: a waiter that queued behind earlier holders stopped a hold
  # that had just started (2026-11-08)
  age=$waited; ts=$(echo "$owner" | cut -s -d' ' -f4)  # `date -d ""` is today's midnight: parse only a non-empty field
  [ -n "$ts" ] && start=$(date -d "$ts" +%s 2>/dev/null) && age=$(( $(date +%s) - start ))
  case "$spid" in ""|*[!0-9]*) ;; *) kill -0 "$spid" 2>/dev/null || gone=true;; esac  # v1 lines: no session field
  if { $gone || { [ $age -ge "$STARVE" ] && ! $starved; }; } && [ "$stopped" != "$owner" ]; then
    echo "[cpu-lock] holder's session gone or 30 min passed; stopping the holders of: $owner" >&2
    pids=$(holders); [ -n "$pids" ] && { kill -TERM $pids 2>/dev/null; sleep "$GRACE"; kill -KILL $pids 2>/dev/null; }
    stopped=$owner
    # Past the wait limit the holder is stopped once, then this waiter queues on the lock itself: a holder that
    # took the lock meanwhile is a live job, not a stale one (stopping every new holder starved everyone, 2026-10-22).
    [ $age -ge "$STARVE" ] && { starved=true; flock 9; break; }
  fi
  sleep "$POLL"; waited=$((waited + POLL))
done
echo "$$ reactive-driving-ai $(session_pid) $(date +%FT%T) $what" > "$OWNER"
timeout 1800 "$@"; rc=$?
: > "$OWNER"
exit $rc
