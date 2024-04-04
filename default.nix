{ lib
, llvmPackages_17
, cmake }:

llvmPackages_17.stdenv.mkDerivation rec {
  pname = "INSTINCT";
  version = "0.1.0";

  src = ./.;

  cmakeFlags = [
    "-Bbuild/Release"
    "-S."
    "-DCMAKE_BUILD_TYPE=Release"
    "-DCMAKE_TOOLCHAIN_FILE=build/Release/generators/conan_toolchain.cmake"
    "-DENABLE_MAIN=ON"
    "-DENABLE_TESTING=OFF"
    "-DENABLE_DOXYGEN=OFF"
    "-DLOG_LEVEL=INFO"
    "-DENABLE_CLANG_TIDY=OFF"
    "-DENABLE_CPPCHECK=OFF"
    "-DENABLE_INCLUDE_WHAT_YOU_USE=OFF"
  ];

  meta = with lib; {
    homepage = "https://github.com/UniStuttgart-INS/INSTINCT";
    description = ''
      INSTINCT - INS Toolkit for Integrated Navigation Concepts and Training";
    '';
    # licencse = licenses.mpl2;
    # platforms = with platforms; linux ++ darwin;
    # maintainers = [  ];
  };
}