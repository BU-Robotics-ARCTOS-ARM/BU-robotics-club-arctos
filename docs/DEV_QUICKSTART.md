# Quickstart — Development Setup

> This is my minimal recommended Python development setup for this repo. These are just recommendations. Use whatever tools and settings you prefer. I cannot guarantee nothing breaks, especially on Windows. I take no responsibility for damage.


## 1. Open a terminal

You need a command-line interface to run all the setup commands that follow.

Open a terminal (Linux/macOS) or PowerShell (Windows).

## 2. Install Git and clone the repo

Git is version control software. It is like Google Drive but for code. It lets you download the project and track changes. Cloning creates a local copy of a repository on your computer. A repository (repo) is basically a project folder that Git tracks, containing the code, files, and the full history of changes.

Install Git if you don't have it already: [git-scm.com/downloads](https://git-scm.com/downloads)

> **Prefer a GUI?** You can use [GitHub Desktop](https://desktop.github.com/) or [GitKraken](https://www.gitkraken.com/) instead of the command line for cloning, committing, and pushing. You'll still need Git installed for `uv` and other tools to work.

```bash
git clone https://github.com/hben09/BU-robotics-club-arctos.git
cd BU-robotics-club-arctos
```

## 3. Install uv and sync dependencies

`uv` is a Python tool that replaces `pip` and `venv`. It ensures everyone uses the same Python version and packages, so code runs the same on every machine.

### Linux / macOS

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Windows (PowerShell)

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

Restart your terminal after installing, then run:

```bash
uv sync
```

This creates a virtual environment (`.venv/`) and installs all project dependencies.

## 4. Verify the setup

Run a quick check to make sure everything installed correctly.

```bash
uv run python --version
```

If this prints a Python version, you're good to go.

## 5. Install and configure VS Code

VS Code is one of the most popular code editors. The Python extension adds language support (autocomplete, debugging, etc.). Ruff is a Python linter and formatter, basically automatically makes the code clean and consistent.

1. Download and install VS Code: [code.visualstudio.com](https://code.visualstudio.com/)
2. Install the [Python extension](https://marketplace.visualstudio.com/items?itemName=ms-python.python). Open VS Code, go to the Extensions sidebar (`Ctrl+Shift+X`), search "Python", and install the one by Microsoft.
3. Install the [Ruff extension](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff). Search "Ruff" in Extensions and install it.
4. Open the project folder in VS Code (`File > Open Folder`).
5. Select the Python interpreter: `Ctrl+Shift+P` > **Python: Select Interpreter** > choose the `.venv` interpreter (`.venv/bin/python` on Linux/macOS, `.venv\Scripts\python.exe` on Windows).
   - This tells VS Code which Python to use for running code, autocomplete, and debugging. It should point to the virtual environment so you get the correct packages.
6. Recommended workspace settings. Create `.vscode/settings.json`:
   - These settings tell VS Code to auto-format your Python files with Ruff on save and to use pytest for running tests.

```json
{
  "python.defaultInterpreterPath": "${workspaceFolder}/.venv/bin/python",
  "[python]": {
    "editor.defaultFormatter": "charliermarsh.ruff",
    "editor.formatOnSave": true
  },
  "python.testing.pytestEnabled": true
}
```

> **Windows:** change the interpreter path to `${workspaceFolder}/.venv/Scripts/python.exe`.


---

This setup should be enough to get going. Every person will have their own preference in tools/softwares.