# Model optimization skill (simplify-robot-model)

A Claude Code skill that turns CAD robots into low-poly, game-ready models for MoSimulator, without visible quality
loss. It drives a headless Blender toolkit (`toolkit/`) and the team asset library
([MoSim-Modding-Fun/Asset-Library](https://github.com/MoSim-Modding-Fun/Asset-Library)), and it carries the house rules
and the lessons from many past robots: what to ask before starting, how to run Blender without crashing the machine,
how to check the result, and what to deliver.

## Install

```bash
git clone git@github.com:MoSim-Modding-Fun/model-optimization-skill.git
cd model-optimization-skill
./install.sh            # Linux / macOS / Git Bash
```

On Windows from cmd or PowerShell: `install.cmd` (or `python install.py`).

The installer asks where your asset library clone is (or clones it with `--clone-library`), where your robot configs
should live, and whether you maintain the asset library. It links the skill into `~/.claude/skills/`, writes
`~/.config/simplify-robot-model/config.json`, and checks the tools below. Re-run it to change settings; `git pull`
updates the skill.

Non-interactive: `./install.sh --library ~/MoSim/Asset-Library --robots ~/MoSim/simplify-robots --robots-root ~/MoSim/robots --yes`
(add `--blender <path to blender>` if Blender isn't found automatically).

## Requirements

- Windows 10/11, macOS or Linux, with Claude Code (on Windows it runs commands in Git Bash: install Git for Windows).
- Blender 5.x, Python 3.8+ (on Windows from python.org or `winget install Python.Python.3.12`; the `python3` that
  opens the Microsoft Store doesn't count), ImageMagick 7 (`magick`), git and git-lfs (for the asset library).
- The resource guard (`scripts/guard.py`) caps every job's memory: systemd user scopes on Linux, job objects on Windows,
  a polling watchdog on macOS. Without a cap, a runaway job can take the whole machine down.

## Use

In Claude Code, ask in plain words: "simplify the robot in `~/MoSim/robots/1234 2026`", "make this CAD cheaper to
render", "build a library asset for this swerve module". The skill works out what it can from the files, tells you what it assumed, then runs.

## Layout

| Path | What |
|---|---|
| `SKILL.md` | The workflow Claude follows. |
| `references/` | Intake questions, house rules, input types, asset library, building assets, toolkit reference, QA, delivery, gotchas. |
| `scripts/guard.py` (+ `guard.sh` / `bl.sh`, `guard.cmd` / `bl.cmd`) | The resource guard every process runs through. |
| `toolkit/` | The Blender scripts (`pipeline.py` and friends). `toolkit/robots/rexample.py` is the robot-config template; your own robot configs live in the folder you gave the installer. |
| `install.py` (+ `install.sh`, `install.cmd`) | Installer. |

## Settings

`~/.config/simplify-robot-model/config.json` (or the environment variables in brackets):

| Key | What |
|---|---|
| `asset_library` (`SIMPLIFY_ASSET_LIBRARY`) | The asset library clone (holds `Components/` and `Materials/`). |
| `robot_config_dirs` (`SIMPLIFY_ROBOT_DIRS`) | Folders with your `r<name>.py` robot configs. |
| `robots_root` (`SIMPLIFY_ROBOTS_ROOT`) | Where robot folders live (informational). |
| `blender` (`SIMPLIFY_BLENDER`) | The Blender executable, if it isn't on PATH or in the usual install folder. |
| `unity` (`SIMPLIFY_UNITY`) | The Unity editor, only for drop-in checks. |
| `font`, `font_bold` | TrueType files for image labels, if the defaults aren't found. |
| `library_maintainer` | `true` only for whoever commits to the asset library; everyone else's sessions never change the library's git. |

## Contributing lessons

When a job teaches something new, add it to the skill as a lesson that stands on its own (no robot or team names), and
keep changed toolkit defaults opt-in until they've been tested on more than one robot.
