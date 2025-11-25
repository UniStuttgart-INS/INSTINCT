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
              gcovr
              mold

              lldb
              libcxx
              clang-tools
              clang

              gcc

              # ccache is currently incompatible with clang
              # ccache
              # ccacheStdenv

              doxygen
              texliveFull
              graphviz
              mscgen
              dia
              ghostscript
              pdf2svg

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
            LD_LIBRARY_PATH = with pkgs;
              lib.makeLibraryPath [
                libGL
                glfw
                libGLU
                stdenv.cc.cc.lib
                gperftools
              ];
          };
        }
      );
    };
}
