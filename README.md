# NavSoS - Navigation Software Stuttgart

Navigation Software of the Institut of Navigation (University of Stuttgart)

## Description

This software provides real-time and post processing functionality for navigational tasks. It can read from sensors and fuse together the data. It can fuse GNSS data with IMU data and do advanced functions like RTK, RAIM, ...

## Getting Started

### Dependencies

* Needed:
    * [cmake](https://cmake.org/) for building the project
    * [make](https://www.gnu.org/software/make/) for building the project
    * [clang-format](https://clang.llvm.org/docs/ClangFormat.html) for code formatting
    * C++ compiler (e.g. [gcc](https://gcc.gnu.org/), [clang](https://clang.llvm.org/)) for compiling the project
    * [spdlog](https://github.com/gabime/spdlog) Fast C++ logging library
    * [docopt](https://github.com/docopt/docopt.cpp) Command line arguments parser
* Optional:
    * [ccache](https://ccache.dev/) to speed up recompilation
    * [googletest](https://github.com/google/googletest) for testing
    * [valgrind](http://valgrind.org/) for CPU profiling & leak detection
    * [kcachegrind](http://kcachegrind.sourceforge.net) for Visualization of Performance Profiling Data
    * [doxygen](http://www.doxygen.nl/) for Documentation creation
* Potentially (might be used in the future):
    * [Boost](https://www.boost.org/) library

### Installing

ArchLinux:
```
# Needed
pacman -S base-devel cmake clang docopt
trizen -S spdlog-git # (use AUR version, as community version is broken)

# Optional
pacman -S ccache gtest valgrind kcachegrind doxygen

# Potentially
pacman -S boost boost-libs
```

Ubuntu:
```
!!! Someone with Ubuntu should provide commands here !!!
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

Recommended changes to the User's ```settings.json``` (not the project .vscode/settings.json)
```
"editor.formatOnType": true,
"doxdocgen.generic.authorEmail": "your.name@nav.uni-stuttgart.de",
"doxdocgen.generic.authorName": "Y. Name",
```

Recommended changes to the User's ```keybindings.json```
```
// Place your key bindings in this file to override the defaultsauto[]
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
| ```F6``` | Run Task: ```DEBUG: Build project```       | Default build |
| ```F7``` | Run Task: ```DEBUG: Build & run project``` | Default test  |
| ```F8``` | Open Task List                             |               |

## Help

The help function can be shown by calling the binary with the ```-h | --help``` parameter
```
navsos --help
```

## Authors

* [M.Sc. Thomas Topp](mailto:thomas.topp@nav.uni-stuttgart.de?subject=[GitLab/NavSoS]%20)
* [M.Sc. Tomke Lambertus](mailto:tomke.lambertus@nav.uni-stuttgart.de?subject=[GitLab/NavSoS]%20)
* [M.Sc. Rui Wang](mailto:rui.wang@nav.uni-stuttgart.de?subject=[GitLab/NavSoS]%20)

## Version History

```This is only a placeholder so far...```
* 0.2
    * Various bug fixes and optimizations
    * See [commit change]() or See [release history]()
* 0.1
    * Initial Release

## License

```This is only a placeholder so far...```

This project is licensed under the [NAME HERE] License - see the LICENSE.md file for details

## Acknowledgments

```This is only a placeholder so far...```

Inspiration, code snippets, etc.
* [awesome-readme](https://github.com/matiassingers/awesome-readme)
* [PurpleBooth](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2)
* [dbader](https://github.com/dbader/readme-template)
* [zenorocha](https://gist.github.com/zenorocha/4526327)
* [fvcproductions](https://gist.github.com/fvcproductions/1bfc2d4aecb01a834b46)