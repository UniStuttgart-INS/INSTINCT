# NavSoS - Navigation Software Stuttgart

Navigation Software of the Institut of Navigation (University of Stuttgart)

## Description

This software provides real-time and post processing functionality for navigational tasks. It can read from sensors and fuse together the data. It can fuse GNSS data with IMU data and do advanced functions like RTK, RAIM, ...

The software consists of 2 parts
* Main console program "```navsos```": Reads in instructions from boost program options config files
* GUI program "```navsos-gui```": Data flow programming Qt5 GUI which exports the config file for the main program. It has main 2 advantages:
    * Completely automatically generated. The GUI Code needs no maintenance, it is generated from the Node Interfaces which are provided by the main program.
    * Completely optional because the boost program option config files can be provided manually.

## Getting Started

### Dependencies

* Needed:
    * [cmake](https://cmake.org/) A cross-platform open-source make system
    * [make](https://www.gnu.org/software/make/) GNU make utility to maintain groups of programs
    * [clang-format](https://clang.llvm.org/docs/ClangFormat.html) Code formatting Tool
    * C++ compiler ([clang](https://clang.llvm.org/) is recommended, but others work as well) for compiling the project
* Optional:
    * [Conan](https://conan.io) A distributed, open source, C/C++ package manager
    * [gnuplot](http://www.gnuplot.info/) Plotting package. Needed if any plotting is intended
    * [ccache](https://ccache.dev/) Compiler cache that speeds up recompilation by caching previous compilations
    * [valgrind](http://valgrind.org/) CPU profiling & leak detection
    * [kcachegrind](http://kcachegrind.sourceforge.net) Visualization of Performance Profiling Data
    * [doxygen](http://www.doxygen.nl/) Documentation system for C++, C, Java, IDL and PHP
    * [clang-tidy](https://clang.llvm.org/extra/clang-tidy/) Clang-based C++ "linter" tool
    * [cppcheck](http://cppcheck.sourceforge.net/) A tool for static C/C++ code analysis
* Libraries (Install yourself and change cmake link targets or let them automatically be installed by Conan):
    * [spdlog](https://github.com/gabime/spdlog) Fast C++ logging library
    * [fmt](https://github.com/fmtlib/fmt) A modern formatting library https://fmt.dev
    * [Boost](https://www.boost.org/) Free peer-reviewed portable C++ source libraries
    * [Eigen](http://eigen.tuxfamily.org) C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms
    * [Catch2](https://github.com/catchorg/Catch2) Modern, C++-native, header-only, test framework for unit-tests, TDD and BDD
* GUI (optional):
    * [Qt](https://www.qt.io/) A cross-platform application and UI framework

### Development Environment Setup

Most library dependencies are managed by Conan.io, so you just need to install the basics.

#### ArchLinux:
```
# Needed
sudo pacman -S base-devel cmake clang
trizen -S conan # AUR package

# Optional
sudo pacman -S ccache doxygen gnuplot cppcheck

# GUI (optional)
sudo pacman -S qt5-base

# Profiling (optional)
sudo pacman -S valgrind kcachegrind
```

#### Ubuntu 20.04:
```
# Needed
sudo apt update
sudo apt upgrade -y
sudo apt install -y build-essential clang clang-tidy cmake python3-pip
sudo apt install -y gcc-10 g++-10
sudo ln -sf /usr/bin/gcc-10 /usr/bin/gcc
sudo ln -sf /usr/bin/g++-10 /usr/bin/g++
pip3 install conan --user

# Optional
sudo apt install ccache doxygen gnuplot gnuplot-x11 cppcheck

# GUI (optional)
sudo apt install qt5-default 

# Profiling (optional)
sudo apt install valgrind kcachegrind
```

#### Raspbian:
Raspbian has old versions of clang and cmake in its repository, so we need to install them manually.
```
# Remove clang and cmake
sudo apt-get remove clang cmake

# Get Clang 10 (clang-tidy included) and update the path
wget https://github.com/llvm/llvm-project/releases/download/llvmorg-10.0.0/clang+llvm-10.0.0-armv7a-linux-gnueabihf.tar.xz
tar -xvf clang+llvm-10.0.0-armv7a-linux-gnueabihf.tar.xz
rm clang+llvm-10.0.0-armv7a-linux-gnueabihf.tar.xz
mv clang+llvm-10.0.0-armv7a-linux-gnueabihf clang_10.0.0
sudo mv clang_10.0.0 /usr/local
echo 'export PATH=/usr/local/clang_10.0.0/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/clang_10.0.0/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc

# Install cmake
sudo apt-get install build-essential python3-pip
pip3 install cmake

# Install conan
pip3 install conan --user
```
Optional dependencies and GUI, see Ubuntu

#### MacOS
```
# Basic
xcode-select --install
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
brew update

# Needed
brew install cmake llvm conan
ln -s "$(brew --prefix llvm)/bin/clang-format" "/usr/local/bin/clang-format"
ln -s "$(brew --prefix llvm)/bin/clang-tidy" "/usr/local/bin/clang-tidy"

# Optional
brew install ccache gnuplot doxygen cppcheck

# GUI (optional)
brew install qt
```

#### Windows 10 (WSL):
[Windows Subsystem for Linux Installation Guide for Windows 10](https://docs.microsoft.com/de-de/windows/wsl/install-win10):

PowerShell (Administrator):
```
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart

dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```
* Restart your computer
* Download & Install the Linux kernel update package [WSL2-Linux-Kernel for x64 computer](https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi)
* Set WSL 2 as your default version: ```wsl --set-default-version 2```
* Next open the Microsoft Store and install [Ubuntu](https://www.microsoft.com/de-de/p/ubuntu/9nblggh4msv6)
* Launch Ubuntu and create a user account and password
* Follow the instructions for Ubuntu here in the Readme

[VSCode Developing in WSL](https://code.visualstudio.com/docs/remote/wsl):
* Install [Visual Studio Code](https://code.visualstudio.com/download) on the Windows side (not in WSL)
* Install the [Remote Development extension pack](https://aka.ms/vscode-remote/download/extension)
* Navigate with Linux terminal to your source code folder and type ```code .```

[Get the GUI to work](https://github.com/microsoft/WSL/issues/4793#issuecomment-577232999):
* Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
* Run with these settings:
    * Multiple windows, Display number: -1
    * Start no client
    * Disable access control
* Hover with your mouse over the system tray icon and verify that it says ```hostname:0.0```
* Find your host IP address (run in windows console: ```ipconfig```), e.g. 192.168.1.3
* In the ```.bash_profile``` add the following and replace the ip address there:
```
export DISPLAY=192.168.1.3:0.0
export LIBGL_ALWAYS_INDIRECT=1
```

### VSCode Configuration

It is strongly recommended to use [Visual Studio Code](https://code.visualstudio.com/) as IDE, as the needed project files are provided in the ```.vscode``` folder.

Recommended plugins for working with this project
* [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools): C/C++ IntelliSense, debugging, and code browsing.
* [CMake](https://marketplace.visualstudio.com/items?itemName=twxs.cmake): CMake langage support for Visual Studio Code
* [Doxygen Documentation Generator](https://marketplace.visualstudio.com/items?itemName=cschlosser.doxdocgen): Automatic Doxygen generation by typing ```/** + [Enter]```
* [Todo Tree](https://marketplace.visualstudio.com/items?itemName=Gruntfuggly.todo-tree): Show TODO, FIXME, etc. comment tags in a tree view
* [Code Spell Checker](https://marketplace.visualstudio.com/items?itemName=streetsidesoftware.code-spell-checker): Spelling checker for source code
* [Log File Highlighter](https://marketplace.visualstudio.com/items?itemName=emilast.LogFileHighlighter): Adds color highlighting to log files to make it easier to follow the flow of log events and identify problems.
* [Clang-Tidy](https://marketplace.visualstudio.com/items?itemName=notskm.clang-tidy) Integrates clang-tidy into VS Code
* [cmake-format](https://marketplace.visualstudio.com/items?itemName=cheshirekow.cmake-format) Format listfiles so they don't look like crap

Recommended changes to the User's ```settings.json``` (**not** the project .vscode/settings.json)
```
"editor.formatOnType": true,
"doxdocgen.generic.authorEmail": "your.name@nav.uni-stuttgart.de",
"doxdocgen.generic.authorName": "Y. Name",
```

Recommended changes to the User's ```keybindings.json```
```
[
    {
        "key": "f6",
        "command": "workbench.action.tasks.build",
        "when": "!inDebugMode"
    },
    {
        "key": "f8",
        "command": "-editor.action.marker.nextInFiles",
        "when": "editorFocus && !editorReadonly"
    },
    {
        "key": "f8",
        "command": "workbench.action.tasks.runTask"
    }
]
```

### Executing the program

| Hotkey   | Action                                     | Default       |
| :------: | :----------------------------------------- | ------------- |
| ```F5``` | Debug the project                          | Default debug |
| ```F6``` | Run Task: ```DEBUG: Build project```       | Default build |
| ```F7``` | Run Task: ```DEBUG: Build & run project``` | Default test  |
| ```F8``` | Open Task List                             |               |

* To start the GUI, execute the Task ```GUI: Build & run project```
* If you have problems with the build, execute the Task ```CLEAN: Remove build files```
* If you want to provide tests, place them in the ```tests``` directory and execute them with the task ```TEST: Build & run```

## Help

The help function can be shown by calling the binary with the ```-h | --help``` parameter
```
navsos --help
```

## Authors

* [M.Sc. Thomas Topp](mailto:topp@ins.uni-stuttgart.de?subject=[GitLab/NavSoS]%20)
* [M.Sc. Rui Wang](mailto:rui.wang@ins.uni-stuttgart.de?subject=[GitLab/NavSoS]%20)

## Version History

## License

```This is only a placeholder so far...```

This project is licensed under the [TBD] License - see the LICENSE.md file for details