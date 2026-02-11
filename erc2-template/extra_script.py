Import("env")
import shutil
import os
import platform
import sys

IS_WINDOWS = platform.system() == "Windows"

GIT_EXE = "git"
HITCHCOCK_GIT_EXE = r'"C:\Program Files\Git\mingw64\bin\git.exe"'

COMMAND_DELIMITER = "&" if IS_WINDOWS else ";"
REMOVE_DIR = "rmdir /s /q" if IS_WINDOWS else "rm -rf"

LIBRARY_SUBMODULE = "lib"

DEVELOPER_MODE = False
DEVELOPER_FIRMWARE_PATH = (
    "D:\dev\erc2firmware"  # Path to your local firmware repository
)

lib_path = env.Dir("lib").srcnode().abspath
root_path = env.Dir("#").srcnode().abspath


def updateFirmware():

    global GIT_EXE

    if DEVELOPER_MODE:
        # Copy the firmware from the developer repository to the lib directory
        shutil.copytree(
            DEVELOPER_FIRMWARE_PATH,
            lib_path,
            dirs_exist_ok=True,
            ignore=shutil.ignore_patterns("*.git", "README.md", "LICENSE"),
        )
    else:
        # If on Windows, check if on a Hitchcock computer

        if IS_WINDOWS:
            git_in_path = os.system("git -h >nul 2>&1") == 0
            git_in_path = git_in_path or os.system("git help >nul 2>&1") == 0
            if not git_in_path:

                print("GIT NOT IN PATH, ADDING")

                # TODO: Smart search for git in Program Files
                # Check and see if git is where it would be on a Hitchcock computer
                if os.path.exists(HITCHCOCK_GIT_EXE):
                    GIT_EXE = HITCHCOCK_GIT_EXE
                else:
                    print("GIT NOT FOUND. PLEASE INSTALL GIT.")
                    sys.exit(1)

        # General user, update the submodule
        command = f'cd "{root_path}" {COMMAND_DELIMITER}'
        command += f"{GIT_EXE} rev-parse --is-inside-work-tree || {GIT_EXE} init {COMMAND_DELIMITER}"  # Init repo if not already done

        # Init repo if not already done
        # If lib is a folder, remove it
        command += (
            f"{GIT_EXE} submodule status {LIBRARY_SUBMODULE} || ("
            + f"{REMOVE_DIR} {LIBRARY_SUBMODULE} {COMMAND_DELIMITER}"
            + f"{GIT_EXE} submodule add --force https://code.osu.edu/fehelectronics/eed_controller/erc2firmware.git {LIBRARY_SUBMODULE} "
            + f"){COMMAND_DELIMITER}"
        )
        command += (
            f"{GIT_EXE} submodule update --init --recursive --checkout --force --remote"
        )
        os.system(command)


updateFirmware()
