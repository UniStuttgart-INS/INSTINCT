#pragma once

#if defined _WINDOWS && defined _BUILD_DLL
    #if defined proglib_cpp_EXPORTS
        #define er_proglib_DLLEXPORT __declspec(dllexport)
    #else
        #define er_proglib_DLLEXPORT __declspec(dllimport)
    #endif
#else
    #define er_proglib_DLLEXPORT
#endif

#if defined _WINDOWS && defined _BUILD_DLL
    #if defined proglib_cpp_graphics_EXPORTS
        #define ub_proglib_graphics_DLLEXPORT __declspec(dllexport)
    #else
        #define ub_proglib_graphics_DLLEXPORT __declspec(dllimport)
    #endif
#else
    #define ub_proglib_graphics_DLLEXPORT
#endif
