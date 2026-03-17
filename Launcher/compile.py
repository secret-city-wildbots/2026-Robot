import subprocess
import sys
import os

# -------- CONFIG --------
SCRIPT_NAME = "overlay.py"   # name of your script file
EXE_NAME = "overlay"
# ------------------------

def run(cmd):
    print(">>", " ".join(cmd))
    subprocess.check_call(cmd)

def pip_install(package):
    run([sys.executable, "-m", "pip", "install", package])

def main():
    print("Installing dependencies...")

    deps = [
        "PyQt6",
        "PyQt6-WebEngine",
        "pyinstaller"
    ]

    for d in deps:
        pip_install(d)

    print("\nBuilding executable with PyInstaller...")

    run([
        "pyinstaller",
        "--noconfirm",
        "--onefile",
        "--windowed",
        "--name", EXE_NAME,
        "--hidden-import", "PyQt6.QtWebEngineCore",
        "--hidden-import", "PyQt6.QtWebEngineWidgets",
        SCRIPT_NAME
    ])

    print("\nBuild complete.")
    print("Executable located in:")
    print(os.path.join("dist", EXE_NAME + ".exe"))

if __name__ == "__main__":
    main()