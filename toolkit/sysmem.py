# System memory facts for the resource guard and the toolkit, on Linux, Windows and macOS, standard library only (works
# in plain python3 and in Blender's Python).
#   mem()            {"total", "available", "swap_total", "swap_used", "commit_used", "commit_limit"} in bytes
#                    Linux: /proc/meminfo. Windows: physical RAM plus the commit charge and limit (RAM + page file; the
#                    real Windows failure is commit reaching the limit, when allocations fail everywhere); swap is
#                    reported as 0 there, since page-file use is normal on Windows and says nothing about pressure.
#                    macOS: the memory-pressure level for "available", and swap measured against RAM (swapfiles are
#                    added as they fill, so used/total is always high there).
#   io_pressure()    Linux PSI "some avg10" in %, else 0
#   peak_rss_mb()    this process's peak resident memory in MB
#   tree_rss(pid)    memory of a process and its descendants (POSIX; macOS uses the physical footprint, which counts
#                    compressed memory; Windows uses job objects instead)
import os
import subprocess
import sys

WINDOWS = sys.platform.startswith("win")
MACOS = sys.platform == "darwin"


def mem():
    if WINDOWS:
        import ctypes

        class MEMORYSTATUSEX(ctypes.Structure):
            _fields_ = [("dwLength", ctypes.c_ulong), ("dwMemoryLoad", ctypes.c_ulong),
                        ("ullTotalPhys", ctypes.c_ulonglong), ("ullAvailPhys", ctypes.c_ulonglong),
                        ("ullTotalPageFile", ctypes.c_ulonglong), ("ullAvailPageFile", ctypes.c_ulonglong),
                        ("ullTotalVirtual", ctypes.c_ulonglong), ("ullAvailVirtual", ctypes.c_ulonglong),
                        ("ullAvailExtendedVirtual", ctypes.c_ulonglong)]
        st = MEMORYSTATUSEX()
        st.dwLength = ctypes.sizeof(MEMORYSTATUSEX)
        ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(st))
        # the "page file" figures are the commit limit (RAM + page file) and what's left of it
        return {"total": st.ullTotalPhys, "available": st.ullAvailPhys, "swap_total": 0, "swap_used": 0,
                "commit_used": st.ullTotalPageFile - st.ullAvailPageFile, "commit_limit": st.ullTotalPageFile}
    if MACOS:
        total = int(subprocess.run(["sysctl", "-n", "hw.memsize"], capture_output=True, text=True).stdout)
        out = subprocess.run(["vm_stat"], capture_output=True, text=True).stdout
        page = 4096
        pages = {}
        for line in out.splitlines():
            if "page size of" in line:
                page = int(line.split("page size of")[1].split()[0])
            elif ":" in line:
                k, v = line.split(":", 1)
                try:
                    pages[k.strip()] = int(v.strip().rstrip("."))
                except ValueError:
                    pass
        avail = page * sum(pages.get(k, 0) for k in ("Pages free", "Pages inactive", "Pages speculative",
                                                     "Pages purgeable"))
        try:  # the memory-pressure figure ("System-wide memory free percentage" of memory_pressure)
            level = int(subprocess.run(["sysctl", "-n", "kern.memorystatus_level"], capture_output=True,
                                       text=True).stdout)
            if 0 < level <= 100:
                avail = total * level // 100
        except ValueError:
            pass
        sw = subprocess.run(["sysctl", "-n", "vm.swapusage"], capture_output=True, text=True).stdout.split()

        def mb(key):
            try:
                v = sw[sw.index(key) + 2]
                return float(v.rstrip("MG")) * (1024 if v.endswith("G") else 1) * 2 ** 20
            except (ValueError, IndexError):
                return 0.0
        return {"total": total, "available": min(avail, total), "swap_total": total, "swap_used": int(mb("used")),
                "commit_used": None, "commit_limit": None}
    info = {}
    with open("/proc/meminfo") as f:
        for line in f:
            k, v = line.split(":", 1)
            info[k] = int(v.split()[0]) * 1024
    return {"total": info["MemTotal"], "available": info["MemAvailable"], "swap_total": info.get("SwapTotal", 0),
            "swap_used": info.get("SwapTotal", 0) - info.get("SwapFree", 0), "commit_used": None, "commit_limit": None}


def io_pressure():
    try:
        with open("/proc/pressure/io") as f:
            for line in f:
                if line.startswith("some"):
                    return float(line.split()[1].split("=")[1])
    except OSError:
        pass
    return 0.0


def peak_rss_mb():
    if WINDOWS:
        import ctypes
        from ctypes import wintypes

        class PMC(ctypes.Structure):
            _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD),
                        ("PeakWorkingSetSize", ctypes.c_size_t), ("WorkingSetSize", ctypes.c_size_t),
                        ("QuotaPeakPagedPoolUsage", ctypes.c_size_t), ("QuotaPagedPoolUsage", ctypes.c_size_t),
                        ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t), ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                        ("PagefileUsage", ctypes.c_size_t), ("PeakPagefileUsage", ctypes.c_size_t)]
        pmc = PMC()
        pmc.cb = ctypes.sizeof(PMC)
        k32, psapi = ctypes.WinDLL("kernel32"), ctypes.WinDLL("psapi")
        k32.GetCurrentProcess.restype = wintypes.HANDLE
        psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(PMC), wintypes.DWORD]
        psapi.GetProcessMemoryInfo.restype = wintypes.BOOL
        if not psapi.GetProcessMemoryInfo(k32.GetCurrentProcess(), ctypes.byref(pmc), pmc.cb):
            return 0.0
        return pmc.PeakWorkingSetSize / 2 ** 20
    import resource
    r = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return r / 2 ** 20 if MACOS else r / 1024  # bytes on macOS, KB on Linux


def _mac_footprint():
    """proc_pid_rusage(pid) -> phys_footprint in bytes (macOS; counts compressed memory, unlike RSS), or None."""
    try:
        import ctypes

        class RUSAGE_V0(ctypes.Structure):
            _fields_ = [("ri_uuid", ctypes.c_uint8 * 16)] + [(n, ctypes.c_uint64) for n in (
                "ri_user_time", "ri_system_time", "ri_pkg_idle_wkups", "ri_interrupt_wkups", "ri_pageins",
                "ri_wired_size", "ri_resident_size", "ri_phys_footprint", "ri_proc_start_abstime",
                "ri_proc_exit_abstime")]
        lib = ctypes.CDLL("/usr/lib/libproc.dylib")
        lib.proc_pid_rusage.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(RUSAGE_V0)]
        lib.proc_pid_rusage.restype = ctypes.c_int

        def footprint(pid):
            r = RUSAGE_V0()
            return r.ri_phys_footprint if lib.proc_pid_rusage(pid, 0, ctypes.byref(r)) == 0 else None
        return footprint
    except (OSError, AttributeError):
        return None


def tree_rss(pid):
    """Bytes held by pid and all its descendants (POSIX `ps` RSS; macOS physical footprint where readable)."""
    out = subprocess.run(["ps", "-A", "-o", "pid=", "-o", "ppid=", "-o", "rss="], capture_output=True, text=True).stdout
    kids, rss = {}, {}
    for line in out.splitlines():
        try:
            p, pp, r = (int(x) for x in line.split()[:3])
        except ValueError:
            continue
        kids.setdefault(pp, []).append(p)
        rss[p] = r * 1024
    fp = _mac_footprint() if MACOS else None
    total, todo, seen = 0, [pid], set()
    while todo:
        p = todo.pop()
        if p in seen:
            continue
        seen.add(p)
        f = fp(p) if fp else None
        total += f if f is not None else rss.get(p, 0)
        todo += kids.get(p, [])
    return total


if __name__ == "__main__":
    m = mem()
    print({k: round(v / 2 ** 30, 2) for k, v in m.items() if v is not None}, "GB; io pressure", io_pressure())
