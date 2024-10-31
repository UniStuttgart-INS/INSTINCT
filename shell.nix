# Use VSCode Extension: mkhl.direnv

{ callPackage, fetchFromGitHub, lib, stdenv,
  cmake, conan, gdb, lldb, gcovr, mold, clang-tools_17, llvmPackages_17, gcc13, gcc13Stdenv, libcxx,
  doxygen, texlive, graphviz, ghostscript, pdf2svg, ccache, ccacheStdenv, llvmPackages_latest, gperftools, gv, valgrind, kdePackages,
  xorg, glfw, gl3w, libGLU, libunwind, libGL, xclip, wl-clipboard
}:

let
  mainPkg = callPackage ./default.nix { };
in
mainPkg.overrideAttrs (oa: {
    # stdenv = pkgs.llvmPackages_16.stdenv;
    nativeBuildInputs = [
      cmake
      conan
      gdb
      lldb
      gcovr
      mold
      clang-tools_17 # clang-format, clang-tidy
      gcc13
      gcc13Stdenv
      doxygen
      texlive.combined.scheme-full
      graphviz
      ghostscript
      pdf2svg

      ccache
      ccacheStdenv
      llvmPackages_latest.libcxxClang
      gv
      valgrind
      kdePackages.kcachegrind

      xclip
      wl-clipboard
    ] ++ (oa.nativeBuildInputs or [ ]);
    buildInputs = [
      xorg.libX11.dev
      glfw
      gl3w
      libGLU
      libunwind
      gperftools
    ] ++ (oa.buildInputs or [ ]);
    LD_LIBRARY_PATH = "${libGL}/lib"
                      + ":${glfw}/lib"
                      + ":${libGLU}/lib"
                      + ":${stdenv.cc.cc.lib}/lib"
                      + ":${libcxx}/lib"
                      + ":${gperftools}/lib";
})