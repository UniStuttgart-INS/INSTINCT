# Use VSCode Extension: mkhl.direnv

with import <nixpkgs> {};

pkgs.mkShell {
    # stdenv = pkgs.llvmPackages_16.stdenv;
    nativeBuildInputs = with pkgs; [
      cmake
      conan
      gdb
      lldb
      gcovr
      mold
      clang-tools_16 # clang-format, clang-tidy
      gcc12
      gcc12Stdenv
      (doxygen.overrideAttrs (oldAttrs: rec {
          version = "1.9.8";
          src = fetchFromGitHub {
            owner = "doxygen";
            repo = "doxygen";
            rev = "Release_${lib.replaceStrings [ "." ] [ "_" ] version}";
            sha256 = "sha256-uQ1Fl2kmY7qmzy34NOmZCgPxVGwmqRqDvV6yEab5P4w=";
          };
        })
      )
      texlive.combined.scheme-full
      graphviz
      ghostscript
      pdf2svg

      ccache
      ccacheStdenv
      llvmPackages_latest.libcxxClang # When using 'gcc' this needs to be commented out
      gperftools
    ];
    buildInputs = with pkgs; [
      xorg.libX11.dev
      glfw
      gl3w
      libGLU
      libunwind
      gperftools
    ];
    LD_LIBRARY_PATH = "${pkgs.libGL}/lib:${pkgs.glfw}/lib:${pkgs.libGLU}/lib:${stdenv.cc.cc.lib}/lib:${pkgs.llvmPackages_16.libcxxabi}/lib:${pkgs.llvmPackages_16.libcxx}/lib:${pkgs.gperftools}/lib";
}
