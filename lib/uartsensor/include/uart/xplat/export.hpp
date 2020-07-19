#pragma once

#if defined _WINDOWS && defined _BUILD_DLL
    #if defined proglib_cpp_EXPORTS
        #define proglib_DLLEXPORT __declspec(dllexport)
    #else
        #define proglib_DLLEXPORT __declspec(dllimport)
    #endif
#else
    #define proglib_DLLEXPORT
#endif

#if defined _WINDOWS && defined _BUILD_DLL
    #if defined proglib_cpp_graphics_EXPORTS
        #define proglib_graphics_DLLEXPORT __declspec(dllexport)
    #else
        #define proglib_graphics_DLLEXPORT __declspec(dllimport)
    #endif
#else
    #define proglib_graphics_DLLEXPORT
#endif
