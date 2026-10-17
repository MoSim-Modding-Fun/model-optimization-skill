#!/bin/bash
# Resource guard for EVERY command a simplification session runs: Blender, Python helpers, big greps or parsers.
# Owner, 2026-10-15: "Make sure ram and cpu usage doesnt cause anything to crash"; "total ram usage shouldnt exceed
# like 80 or 90%". At 00:03 that day systemd-oomd killed the whole Claude desktop app, with every session in it,
# because RAM and zram swap were both above 90 %. Anything started without its own systemd scope counts against the
# app's cgroup, and oomd kills that cgroup as one unit.
#
#   LOCK=1 MEM=4G CPU=800% guard.sh <command...>     heavy job (pipeline, vis, render pass, batch cleanup)
#   LOCK=0 MEM=2G guard.sh python3 helper.py ...      small helper (still gets its own scope and the RAM gate)
#   SLOTS=2 LOCK=1 ...                                allow a second heavy job on the box (only with RAM to spare)
# The command runs in the background (`&`): stdin heredocs don't reach it, so write helper scripts to files. Bound
# every loop that trusts a count read from a file: a runaway parser is what got the app killed.
#
# 1. LOCK=1: takes the machine-wide lock ~/.cache/mosim-heavy.lock (flock), shared with every other robot session, so
#    only one heavy job runs on the box at a time. Waits up to 2 h, then exits 75.
# 2. Start gate: waits until MemAvailable - MEM >= 15 % of MemTotal (the box stays <= ~85 % used at this job's cap)
#    AND swap (zram, which lives in RAM) is <= 40 % used. Also waits while IO pressure is very high.
# 3. Runs in its own systemd scope: MemoryMax=MEM, MemorySwapMax=0 and no MemoryHigh (a job over its cap is
#    OOM-killed at once and alone, not left throttled in zram for minutes), CPUQuota=CPU, nice 15, IO class 2/7.
# 4. Watchdog: kills the job if the system passes 90 % RAM or 60 % swap.
# Exit codes: the command's own, 75 = gave up waiting, 137 = killed by the watchdog or by its MemoryMax.
MEM=${MEM:-2G}; CPU=${CPU:-400%}; LOCK=${LOCK:-0}
mem_kb=$(( ${MEM%G} * 1024 * 1024 ))
mi() { awk -v k="$1:" '$1 == k {print $2}' /proc/meminfo; }
total_kb=$(mi MemTotal); swap_kb=$(mi SwapTotal)
swap_used() { echo $(( swap_kb - $(mi SwapFree) )); }
io_some() { awk '/^some/ {split($2, a, "="); printf "%d", a[2]}' /proc/pressure/io 2>/dev/null || echo 0; }

if [ "$LOCK" = 1 ]; then
  # slot 1 = ~/.cache/mosim-heavy.lock (the lock every robot session's guard takes); SLOTS=2 also tries
  # ~/.cache/mosim-heavy.2.lock (an older two-slot wrapper used it too), so at most two guarded heavy jobs run at once. The RAM gate below still
  # has to pass for each. Default 1: the owner's rule is "nothing may crash", not "be fast".
  mkdir -p ~/.cache; exec 8>~/.cache/mosim-heavy.lock 9>~/.cache/mosim-heavy.2.lock
  t=0
  while :; do
    if flock -n 8; then exec 9>&-; break; fi
    if [ "${SLOTS:-1}" -ge 2 ] && flock -n 9; then exec 8>&-; break; fi
    [ $t -eq 0 ] && echo "guard: waiting for a heavy-job slot (another session's job holds it)" >&2
    sleep 10; t=$((t + 10)); [ $t -ge 7200 ] && { echo "guard: gave up waiting for a slot after 2 h" >&2; exit 75; }
  done
fi

waited=0
while [ $(( $(mi MemAvailable) - mem_kb )) -lt $(( total_kb * 15 / 100 )) ] \
   || [ "$(swap_used)" -gt $(( swap_kb * 40 / 100 )) ] \
   || [ "$(io_some)" -gt 50 ]; do
  [ $waited -eq 0 ] && echo "guard: waiting (avail $(( $(mi MemAvailable) / 1024 )) MB, swap used $(( $(swap_used) / 1024 )) MB, io some $(io_some) %)" >&2
  sleep 20; waited=$((waited + 20))
  [ $waited -ge 7200 ] && { echo "guard: gave up waiting after 2 h" >&2; exit 75; }
done

systemd-run --user --scope -q --expand-environment=no -p MemoryMax=$MEM -p MemorySwapMax=0 -p CPUQuota=$CPU -- \
  nice -n 15 ionice -c 2 -n 7 "$@" 8>&- 9>&- &
PID=$!
while kill -0 $PID 2>/dev/null; do
  if [ "$(mi MemAvailable)" -lt $(( total_kb / 10 )) ] || [ "$(swap_used)" -gt $(( swap_kb * 60 / 100 )) ]; then
    echo "guard: killed: system RAM > 90 % or swap > 60 % (avail $(( $(mi MemAvailable) / 1024 )) MB, swap $(( $(swap_used) / 1024 )) MB)" >&2
    kill -9 $PID 2>/dev/null
  fi
  sleep 2
done
wait $PID
