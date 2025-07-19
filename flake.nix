{
  description = "INSTINCT - INS Toolkit for Integrated Navigation Concepts and Training";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs =
    { self, nixpkgs }:
    let
      forAllSystems = nixpkgs.lib.genAttrs [
        "x86_64-linux"
        "aarch64-linux"
      ];
    in
    {
      devShells = forAllSystems (
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
        in
        {
          default = pkgs.mkShell {
            nativeBuildInputs = with pkgs; [
              cmake
              conan
              gdb
              lldb
              gcovr
              mold
              # clang-tools # clang-format, clang-tidy
              llvmPackages_20.clang-tools
              gcc
              gccStdenv
              doxygen
              texliveFull
              graphviz
              mscgen
              dia
              ghostscript
              pdf2svg

              ccache
              ccacheStdenv
              llvmPackages.libcxxClang
              gv
              valgrind
              kdePackages.kcachegrind

              xclip
              wl-clipboard
            ];
            buildInputs = with pkgs; [
              xorg.libX11.dev
              glfw
              gl3w
              libGLU
              libunwind
              gperftools
            ];
            LD_LIBRARY_PATH =
              with pkgs;
              "${libGL}/lib"
              + ":${glfw}/lib"
              + ":${libGLU}/lib"
              + ":${stdenv.cc.cc.lib}/lib"
              + ":${llvmPackages.libcxx}/lib"
              + ":${gperftools}/lib";
          };
        }
      );
    };
}
