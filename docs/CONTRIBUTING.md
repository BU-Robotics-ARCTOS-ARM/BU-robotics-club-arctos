# Contributing

How to contribute to this project. Read this before writing any code.

---

## Getting Started

1. Read `DEV_QUICKSTART.md` and get your environment working
2. Read `ARCHITECTURE.md` to understand where your code belongs
3. Check the [TODO Board](https://github.com/orgs/BU-Robotics-ARCTOS-ARM/projects/4) for available tasks
4. Assign yourself to a task on the board

---

## Git Workflow

### Branches

We use a simple branch model:

- **`main`** — always working. Never push directly to main.
- **Feature branches** — all work happens here, then gets merged via pull request.

Branch naming:

```
<prefix>/<short-description>

Prefixes:
  feat/  — new functionality
  fix/   — bug fixes
  docs/  — documentation

Examples:
  feat/forward-kinematics
  fix/motor-timeout
  docs/setup-guide
```

Personal test/experiment branches can use any name.

### Workflow

```bash
# 1. Make sure main is up to date
git checkout main
git pull

# 2. Create your branch
git checkout -b feat/can-interface

# 3. Do your work, commit often with clear messages
git add .
git commit -m "Add CAN send/receive with timeout handling"

# 4. Push your branch
git push origin feat/can-interface

# 5. Open a Pull Request on GitHub when ready for review
```

> **Note:** This process is subject to change.

### Commit Messages

Write clear, short commit messages. Describe what you did, not how.

**Good:**
```
Add CRC calculation to motor driver
Fix timeout handling when motor doesn't respond
Add joint angle limits to config
Home all joints in safe order (Z first, then Y, then X)
```

**Bad:**
```
stuff
fixed it
WIP
asdfasdf
changes
```

If a commit is related to a specific issue or TODO item, reference it:
```
Add emergency stop keybinding (closes #12)
```

### Pull Requests

- All changes go through pull requests. Never push directly to main.
- No required reviews — merge your own PR when you're ready.
- Reviews are welcome but optional. Tag a teammate if you want a second look.
- Use **squash merge** to keep `main` history clean (one commit per PR).
- Delete your branch after merging.
- Keep PRs small and focused. One feature or fix per PR.
- Write a short description of what changed and why.

> **Note:** This process is subject to change.

#### GitHub Repo Settings

Set these once in **Settings > General > Pull Requests**:
- Allow **squash merging** only (disable merge commits and rebase)
- Enable **Automatically delete head branches**

---
