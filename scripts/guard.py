#!/usr/bin/env python3
# Resource guard for EVERY command a simplification session runs: Blender, Python helpers, Unity, big parsers. Linux,
# Windows and macOS, standard library only. Owner rule: "Make sure ram and cpu usage doesnt cause anything to crash";
# "total ram usage shouldnt exceed like 80 or 90%". Processes started without their own cap count against whatever
# launched them; on one machine memory pressure once killed the whole Claude desktop app, every session with it.
#
#   python3 guard.py [options] [--] <command...>          (bash wrappers: guard.sh, bl.sh; Windows: guard.cmd, bl.cmd)
#   python3 guard.py --blender [options] [--] <blender args...>    runs the configured Blender with -b -t <threads>
# Options (environment variable in brackets, flags win):
#   --lock / --no-lock [LOCK=1/0]   take the machine-wide heavy-job lock (default: on for --blender, off otherwise)
#   --mem 4G       [MEM]            memory cap for the job (default 4G with --blender, 2G otherwise)
#   --cpu 800%     [CPU]            CPU quota (default 800% with --blender, 400% otherwise; Linux systemd scopes only;
#                                   elsewhere the job just runs at low priority)
#   --threads 8    [THREADS]        Blender threads (--blender)
#   --slots 1      [SLOTS]          allow a second heavy job on the machine (2) when RAM allows
#   --lock-name heavy [LOCK_NAME]   lock file ~/.cache/mosim-<name>.lock (e.g. "git" to serialise git in a shared repo)
#   --timeout 1200 [TIMEOUT]        kill the job (and everything it started) after this many seconds (exit 124)
#   [MIN_FREE=15] [MAX_SWAP=40]     start-gate floor (% of RAM left after the cap) and swap ceiling (%): raise /
#                                   lower them to leave more room for other sessions (never below the defaults)
#   [YIELD_TO=regex] [YIELD_MIN_MB=500]  the owner's priority session: the job does not start while a process whose
#                                   command line matches (and uses >= YIELD_MIN_MB resident) runs, and is killed with
#                                   exit 76 if one starts (Linux; environment only, so the pattern is never in the
#                                   guard's own command line; the guard's ancestors are skipped)
#
# 1. Lock: ~/.cache/mosim-heavy.lock (+ .2 with --slots 2), shared with every robot session on the machine (the same
#    files the older bash guard used, compatible on Linux). Waits up to 2 h, then exits 75.
# 2. Start gate: waits until available RAM minus the cap stays >= 15 % of total, swap use <= 40 % (Linux: of the swap,
#    zram counts as RAM; macOS: of RAM), and Linux IO pressure <= 50 %. Gives up after 2 h (exit 75).
# 3. Cap: Linux = own systemd user scope (MemoryMax, MemorySwapMax=0, CPUQuota, nice 15, ionice); Windows = a job object
#    with a job memory limit (it counts committed memory, a little more than the resident size; allocations past it
#    fail; the job dies with the guard) at below-normal priority; macOS (or Linux without systemd) = its own process
#    group at nice 15, killed when its tree's memory passes the cap.
# 4. Watchdog every 2 s: kills the job if the system passes 90 % RAM, 60 % swap (Linux, macOS) or 97 % of the commit
#    limit (Windows). Exit codes: the command's own, 75 = gave up waiting, 124 = timeout, 127 = command not found,
#    137 = killed by the watchdog or its cap (on Windows also a job that failed after reaching its cap), 76 = killed to
#    make room for a YIELD_TO process (rerun it later).
# stdin is passed through. On Windows bare `python3`/`python` run the guard's own Python and bare `bash` is Git Bash
# (not WSL); a .sh script runs through bash. Bound every loop that trusts a count read from a file: a runaway parser
# killed an app once.
import os
import shutil
import signal
import subprocess
import sys
import time
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "toolkit"))
import sysmem  # noqa: E402

WIN, MAC = sysmem.WINDOWS, sysmem.MACOS
WAIT_S = 7200


def parse(argv):
    o = {"blender": False, "lock": None, "mem": os.environ.get("MEM"), "cpu": os.environ.get("CPU"),
         "threads": os.environ.get("THREADS", "8"), "slots": int(os.environ.get("SLOTS", "1")),
         "lock_name": os.environ.get("LOCK_NAME", "heavy"), "timeout": float(os.environ.get("TIMEOUT", "0")),
         # stricter only: a lower floor or a higher swap ceiling than the defaults is ignored
         "min_free": max(15.0, float(os.environ.get("MIN_FREE", "15"))),
         "max_swap": min(40.0, float(os.environ.get("MAX_SWAP", "40"))),
         "yield_to": os.environ.get("YIELD_TO") or None, "yield_min_mb": float(os.environ.get("YIELD_MIN_MB", "500"))}
    if os.environ.get("LOCK") in ("0", "1"):
        o["lock"] = os.environ["LOCK"] == "1"
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--":
            i += 1
            break
        if a == "--blender":
            o["blender"] = True
        elif a == "--lock":
            o["lock"] = True
        elif a == "--no-lock":
            o["lock"] = False
        elif a in ("--mem", "--cpu", "--threads", "--slots", "--lock-name", "--timeout"):
            v = argv[i + 1]
            o[a[2:].replace("-", "_")] = int(v) if a == "--slots" else float(v) if a == "--timeout" else v
            i += 1
        else:
            break
        i += 1
    cmd = argv[i:]
    if o["lock"] is None:
        o["lock"] = o["blender"]
    if not o["mem"]:
        o["mem"] = "4G" if o["blender"] else "2G"
    if not o["cpu"]:
        o["cpu"] = "800%" if o["blender"] else "400%"
    if o["blender"]:
        import paths
        cmd = [paths.blender(), "-b", "-t", str(o["threads"])] + cmd
    if not cmd:
        sys.exit("guard: no command")
    return o, cmd


def bytes_of(s):
    s = s.strip().upper()
    mult = {"K": 2 ** 10, "M": 2 ** 20, "G": 2 ** 30, "T": 2 ** 40}
    return int(float(s[:-1]) * mult[s[-1]]) if s[-1] in mult else int(s)


def say(*a):
    print("guard:", *a, file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------------------------------- lock
def take_lock(name, slots):
    d = os.path.join(os.path.expanduser("~"), ".cache")
    os.makedirs(d, exist_ok=True)
    files = [os.path.join(d, f"mosim-{name}.lock")] + ([os.path.join(d, f"mosim-{name}.2.lock")] if slots >= 2 else [])
    handles = [open(f, "a+") for f in files]
    t0, told = time.time(), False
    while True:
        for h in handles:
            try:
                if WIN:
                    import msvcrt
                    h.seek(0)
                    msvcrt.locking(h.fileno(), msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl
                    fcntl.flock(h.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                for other in handles:
                    if other is not h:
                        other.close()
                return h  # keep it open (and locked) until the guard exits
            except OSError:
                pass
        if not told:
            say("waiting for a heavy-job slot (another session's job holds it)")
            told = True
        if time.time() - t0 > WAIT_S:
            say("gave up waiting for a slot after 2 h")
            sys.exit(75)
        time.sleep(10)


# ---------------------------------------------------------------------------------------------------- gate
def describe(m):
    s = f"available {m['available'] // 2 ** 20} MB of {m['total'] // 2 ** 20} MB"
    if m.get("commit_limit"):
        return s + f", commit {m['commit_used'] // 2 ** 20} of {m['commit_limit'] // 2 ** 20} MB"
    return s + f", swap used {m['swap_used'] // 2 ** 20} MB"


def priority_running(pattern, min_mb):
    """YIELD_TO: a process whose command line matches `pattern` and holds >= min_mb MB resident (Linux /proc), i.e. the
    owner's priority session is busy. The guard's ancestors (the shell whose command holds the pattern) and the guard's
    own job (a guarded Unity -batchmode run would match "-batchmode") are skipped."""
    if not pattern or not os.path.isdir("/proc"):
        return None
    import re
    rx, page = re.compile(pattern), os.sysconf("SC_PAGE_SIZE")
    stat = {}  # pid -> (ppid, rss pages)
    for d in os.listdir("/proc"):
        if d.isdigit():
            try:
                f = open(f"/proc/{d}/stat").read().rsplit(")", 1)[1].split()
                stat[int(d)] = (int(f[1]), int(f[21]))
            except (OSError, ValueError, IndexError):
                pass
    me, skip = os.getpid(), set()
    p = me
    while p > 1 and p not in skip:  # ancestors
        skip.add(p)
        p = stat.get(p, (0, 0))[0]
    kids = {}
    for pid, (ppid, _) in stat.items():
        kids.setdefault(ppid, []).append(pid)
    todo = [me]
    while todo:  # the guard's own job and everything it started
        p = todo.pop()
        for c in kids.get(p, []):
            if c not in skip:
                skip.add(c)
                todo.append(c)
    for pid, (_, rss) in stat.items():
        rss_mb = rss * page / 2 ** 20
        if pid in skip or rss_mb < min_mb:
            continue
        try:
            cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace").strip()
        except OSError:
            continue
        if rx.search(cmd):
            return f"pid {pid} {cmd[:70]} ({rss_mb:.0f} MB)"
    return None


def wait_for_room(cap, o):
    t0, told = time.time(), False
    while True:
        m = sysmem.mem()
        swap_ok = m["swap_total"] == 0 or m["swap_used"] <= o["max_swap"] / 100 * m["swap_total"]
        busy = priority_running(o["yield_to"], o["yield_min_mb"])
        if (m["available"] - cap >= o["min_free"] / 100 * m["total"] and swap_ok and sysmem.io_pressure() <= 50
                and not busy):
            return
        if not told:
            say(f"waiting ({describe(m)}, io {sysmem.io_pressure():.0f} %" + (f"; priority process {busy}" if busy else "") + ")")
            told = True
        if time.time() - t0 > WAIT_S:
            say("gave up waiting for RAM after 2 h")
            sys.exit(75)
        time.sleep(20)


def system_overloaded():
    m = sysmem.mem()
    if m["available"] < 0.10 * m["total"]:
        return m
    if m["swap_total"] > 0 and m["swap_used"] > 0.60 * m["swap_total"]:
        return m
    if m.get("commit_limit") and m["commit_used"] > 0.97 * m["commit_limit"]:
        return m  # Windows: allocations are about to fail system-wide
    return None


# ---------------------------------------------------------------------------------------------------- Windows job
class WinJob:
    def __init__(self, cap):
        import ctypes
        from ctypes import wintypes
        self.k32 = k32 = ctypes.WinDLL("kernel32", use_last_error=True)
        k32.CreateJobObjectW.restype = wintypes.HANDLE
        k32.CreateJobObjectW.argtypes = [ctypes.c_void_p, wintypes.LPCWSTR]
        k32.SetInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD]
        k32.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
        k32.TerminateJobObject.argtypes = [wintypes.HANDLE, wintypes.UINT]
        k32.QueryInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD,
                                                  ctypes.c_void_p]
        self.ctypes, self.wintypes, self.ok = ctypes, wintypes, False

        class IO_COUNTERS(ctypes.Structure):
            _fields_ = [(n, ctypes.c_ulonglong) for n in ("ReadOperationCount", "WriteOperationCount",
                                                          "OtherOperationCount", "ReadTransferCount",
                                                          "WriteTransferCount", "OtherTransferCount")]

        class BASIC(ctypes.Structure):
            _fields_ = [("PerProcessUserTimeLimit", ctypes.c_longlong), ("PerJobUserTimeLimit", ctypes.c_longlong),
                        ("LimitFlags", wintypes.DWORD), ("MinimumWorkingSetSize", ctypes.c_size_t),
                        ("MaximumWorkingSetSize", ctypes.c_size_t), ("ActiveProcessLimit", wintypes.DWORD),
                        ("Affinity", ctypes.c_size_t), ("PriorityClass", wintypes.DWORD),
                        ("SchedulingClass", wintypes.DWORD)]

        class EXTENDED(ctypes.Structure):
            _fields_ = [("BasicLimitInformation", BASIC), ("IoInfo", IO_COUNTERS),
                        ("ProcessMemoryLimit", ctypes.c_size_t), ("JobMemoryLimit", ctypes.c_size_t),
                        ("PeakProcessMemoryUsed", ctypes.c_size_t), ("PeakJobMemoryUsed", ctypes.c_size_t)]

        # DIE_ON_UNHANDLED_EXCEPTION: a crash after a failed allocation must not sit in an error dialog holding the lock
        JOB_MEMORY, DIE_ON_UNHANDLED_EXCEPTION, KILL_ON_JOB_CLOSE = 0x200, 0x400, 0x2000
        self.EXTENDED = EXTENDED
        self.h = k32.CreateJobObjectW(None, None)
        if not self.h:
            raise OSError(ctypes.get_last_error(), "CreateJobObject failed")
        info = EXTENDED()
        info.BasicLimitInformation.LimitFlags = JOB_MEMORY | DIE_ON_UNHANDLED_EXCEPTION | KILL_ON_JOB_CLOSE
        info.JobMemoryLimit = cap
        if not k32.SetInformationJobObject(self.h, 9, ctypes.byref(info), ctypes.sizeof(info)):  # extended limits
            raise OSError(ctypes.get_last_error(), "SetInformationJobObject failed")

    def add(self, proc):
        self.ok = bool(self.k32.AssignProcessToJobObject(self.h, int(proc._handle)))
        if not self.ok:
            say("could not put the job in a job object (error", self.ctypes.get_last_error(),
                "): no per-job cap, only the watchdog guards it")
        return self.ok

    def resume(self, proc):
        """Start a process created suspended (it went into the job before it could start anything)."""
        nt = self.ctypes.WinDLL("ntdll")
        nt.NtResumeProcess.argtypes = [self.wintypes.HANDLE]
        nt.NtResumeProcess.restype = self.ctypes.c_long
        return nt.NtResumeProcess(int(proc._handle)) == 0

    def peak(self):
        info = self.EXTENDED()
        ok = self.k32.QueryInformationJobObject(self.h, 9, self.ctypes.byref(info), self.ctypes.sizeof(info), None)
        return info.PeakJobMemoryUsed if ok else 0

    def kill(self):
        self.k32.TerminateJobObject(self.h, 137)


def win_resolve(cmd):
    """Windows: CreateProcess searches System32 before PATH (bare `bash` = WSL) and `python3` is usually the Store
    stub. Run bare python with the guard's own Python, bash from Git, and .sh scripts through that bash."""
    if cmd[0].lower().endswith(".sh"):
        cmd = ["bash"] + cmd
    name, base = cmd[0], os.path.basename(cmd[0]).lower()
    if os.path.dirname(name):
        return cmd
    if base in ("python3", "python3.exe", "python", "python.exe"):
        return [sys.executable] + cmd[1:]
    path = None
    if base in ("bash", "bash.exe", "sh", "sh.exe"):
        sysroot = os.path.normcase(os.environ.get("SystemRoot", r"C:\Windows"))
        path = os.pathsep.join(d for d in os.environ.get("PATH", "").split(os.pathsep)
                               if d and not os.path.normcase(os.path.abspath(d)).startswith(sysroot)
                               and "windowsapps" not in d.lower())
    exe = shutil.which(name, path=path)
    if not exe and base in ("bash", "bash.exe"):
        git = shutil.which("git")  # ...\Git\cmd\git.exe -> ...\Git\bin\bash.exe
        cand = git and os.path.join(os.path.dirname(os.path.dirname(git)), "bin", "bash.exe")
        exe = cand if cand and os.path.exists(cand) else None
    return [exe] + cmd[1:] if exe else cmd


# ---------------------------------------------------------------------------------------------------- run
def has_systemd_scope():
    if WIN or MAC or os.environ.get("GUARD_NO_SYSTEMD") == "1":
        return False
    try:
        return subprocess.run(["systemd-run", "--user", "--scope", "-q", "--", "true"], capture_output=True,
                              timeout=20).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def systemd_extra():
    """--expand-environment=no (systemd 254+) keeps a literal $ in the command; older systemd doesn't expand at all."""
    try:
        out = subprocess.run(["systemd-run", "--help"], capture_output=True, text=True, timeout=20).stdout
    except (OSError, subprocess.TimeoutExpired):
        return []
    return ["--expand-environment=no"] if "--expand-environment" in out else []


def interrupt(*_):
    raise KeyboardInterrupt


def popen(cmd, **kw):
    try:
        return subprocess.Popen(cmd, **kw)
    except OSError as e:
        say(f"could not start {cmd[0]}: {e}")
        sys.exit(127)


def main():
    o, cmd = parse(sys.argv[1:])
    cap = bytes_of(o["mem"])
    if not WIN:  # a SIGTERM/SIGHUP to the guard must take the job with it (it runs in its own session/scope)
        for sig in (signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, interrupt)
    lock = take_lock(o["lock_name"], o["slots"]) if o["lock"] else None  # noqa: F841 (held until exit)
    wait_for_room(cap, o)
    job, unit, scoped = None, None, False
    if WIN:
        cmd = win_resolve(cmd)
        flags = 0x00004000 | 0x00000200  # BELOW_NORMAL_PRIORITY_CLASS | CREATE_NEW_PROCESS_GROUP
        try:
            job = WinJob(cap)  # before the child exists, so nothing it starts escapes the cap
        except OSError as e:
            say("job object unavailable:", e, "- no per-job cap, only the watchdog guards this job")
        proc = popen(cmd, creationflags=flags | (0x4 if job else 0))  # CREATE_SUSPENDED
        if job:
            try:
                job.add(proc)
            finally:
                resumed = job.resume(proc)
            if not resumed:
                proc.kill()
                say("could not resume the job's process")
                return 1
    elif has_systemd_scope():
        unit = f"mosim-guard-{os.getpid()}-{uuid.uuid4().hex[:6]}"
        pre = ["systemd-run", "--user", "--scope", "-q", *systemd_extra(), f"--unit={unit}",
               "-p", f"MemoryMax={o['mem']}", "-p", "MemorySwapMax=0", "-p", f"CPUQuota={o['cpu']}", "--",
               "nice", "-n", "15"]
        if shutil.which("ionice"):
            pre += ["ionice", "-c", "2", "-n", "7"]
        proc = popen(pre + cmd)
        scoped = True
    else:
        proc = popen(cmd, start_new_session=True, preexec_fn=lambda: os.nice(15))
    use_job = bool(job and job.ok)

    def kill(reason):
        say("killed:", reason)
        if use_job:
            job.kill()
        elif unit:
            subprocess.run(["systemctl", "--user", "kill", "--signal=SIGKILL", f"{unit}.scope"], capture_output=True)
        elif WIN:
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
        else:
            try:
                os.killpg(proc.pid, 9)
            except OSError:
                pass
        try:
            proc.kill()
        except OSError:
            pass

    killed, timed_out, yielded, t_start = False, False, False, time.time()
    try:
        while proc.poll() is None:
            if o["timeout"] and time.time() - t_start > o["timeout"]:
                kill(f"timeout after {o['timeout']:.0f} s")
                timed_out = True
                break
            m = system_overloaded()
            if m:
                kill(f"the system is out of memory ({describe(m)})")
                killed = True
                break
            busy = priority_running(o["yield_to"], o["yield_min_mb"])
            if busy:
                kill(f"making room for the priority process {busy} (YIELD_TO)")
                yielded = True
                break
            if not (use_job or scoped or WIN) and sysmem.tree_rss(proc.pid) > cap:
                kill(f"the job's memory passed its {o['mem']} cap")
                killed = True
                break
            time.sleep(2)
        rc = proc.wait()
    except KeyboardInterrupt:
        kill("interrupted")
        rc = 130
    finally:
        if unit:  # anything the job left running in its scope (a package manager child) goes with it
            subprocess.run(["systemctl", "--user", "stop", f"{unit}.scope"], capture_output=True)
        elif use_job:
            job.kill()
        elif not WIN:
            try:
                os.killpg(proc.pid, 9)
            except OSError:
                pass
    if timed_out:
        return 124
    if yielded:
        return 76
    if killed or rc in (-9, 137):
        return 137
    if WIN:
        if use_job and rc != 0 and job.peak() >= 0.9 * cap:
            say(f"the job reached its {o['mem']} cap (an allocation failed; exit {rc & 0xFFFFFFFF:#x})")
            return 137
        return rc - (1 << 32) if rc > 0x7FFFFFFF else rc  # keep an NTSTATUS crash code; sys.exit needs a C long
    return rc if rc >= 0 else 128 - rc


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:  # before the job started (waiting for the lock or for RAM)
        sys.exit(130)
