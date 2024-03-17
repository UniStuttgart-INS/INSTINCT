# Use VSCode Extension: mkhl.direnv

{ callPackage, fetchFromGitHub, lib, stdenv,
  cmake, conan, gdb, lldb, gcovr, mold, clang-tools_16, llvmPackages_16, gcc12, gcc12Stdenv,
  doxygen, texlive, graphviz, ghostscript, pdf2svg, ccache, ccacheStdenv, llvmPackages_latest, gperftools,
  xorg, glfw, gl3w, libGLU, libunwind, libGL
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
    ] ++ (oa.nativeBuildInputs or [ ]);
    buildInputs = [
      xorg.libX11.dev
      glfw
      gl3w
      libGLU
      libunwind
      gperftools
    ] ++ (oa.buildInputs or [ ]);
    LD_LIBRARY_PATH = "${libGL}/lib:${glfw}/lib:${libGLU}/lib:${stdenv.cc.cc.lib}/lib:${llvmPackages_16.libcxxabi}/lib:${llvmPackages_16.libcxx}/lib:${gperftools}/lib";
})