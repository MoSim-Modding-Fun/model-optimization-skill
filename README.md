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
./install.sh
```

The installer asks where your asset library clone is (or clones it with `--clone-library`), where your robot configs
should live, and whether you maintain the asset library. It links the skill into `~/.claude/skills/`, writes
`~/.config/simplify-robot-model/config.json`, and checks the tools below. Re-run it to change settings; `git pull`
updates the skill.

Non-interactive: `./install.sh --library ~/MoSim/Asset-Library --robots ~/MoSim/simplify-robots --robots-root ~/MoSim/robots --yes`.

## Requirements

- Linux with systemd user scopes (the resource guard in `scripts/guard.sh` caps every job's memory; without it, jobs
  can take the whole machine down).
- Blender 5.x on `PATH`, python3, ImageMagick (`magick`), git and git-lfs (for the asset library).
- Claude Code.

## Use

In Claude Code, ask in plain words: "simplify the robot in `~/MoSim/robots/1234 2026`", "make this CAD cheaper to
render", "build a library asset for this swerve module". The skill asks its intake questions first, then runs.

## Layout

| Path | What |
|---|---|
| `SKILL.md` | The workflow Claude follows. |
| `references/` | Intake questions, house rules, input types, asset library, building assets, toolkit reference, QA, delivery, gotchas. |
| `scripts/guard.sh`, `scripts/bl.sh` | The resource guard every process runs through. |
| `toolkit/` | The Blender scripts (`pipeline.py` and friends). `toolkit/robots/rexample.py` is the robot-config template; your own robot configs live in the folder you gave the installer. |
| `install.sh` | Installer. |

## Settings

`~/.config/simplify-robot-model/config.json` (or the environment variables in brackets):

| Key | What |
|---|---|
| `asset_library` (`SIMPLIFY_ASSET_LIBRARY`) | The asset library clone (holds `Components/` and `Materials/`). |
| `robot_config_dirs` (`SIMPLIFY_ROBOT_DIRS`) | Folders with your `r<name>.py` robot configs. |
| `robots_root` (`SIMPLIFY_ROBOTS_ROOT`) | Where robot folders live (informational). |
| `library_maintainer` | `true` only for whoever commits to the asset library; everyone else's sessions never change the library's git. |

## Contributing lessons

When a job teaches something new, add it to the skill as a lesson that stands on its own (no robot or team names), and
keep changed toolkit defaults opt-in until they've been tested on more than one robot.
