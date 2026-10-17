@echo off
rem Resource guard for cmd / PowerShell: guard.cmd [--lock] [--mem 4G] [--] <command...>  (see guard.py)
rem Uses the first Python 3.8+ that actually runs (python, then py -3): the Windows Store stub doesn't count.
setlocal
set "PY="
python -c "import sys; sys.exit(sys.version_info < (3, 8))" >nul 2>nul && set "PY=python"
if not defined PY (py -3 -c "import sys; sys.exit(sys.version_info < (3, 8))" >nul 2>nul && set "PY=py -3")
if not defined PY (
  echo guard.cmd: no Python 3.8+ found, tried python and py -3; install it from python.org 1>&2
  exit /b 127
)
%PY% "%~dp0guard.py" %*
exit /b %errorlevel%
