#pragma once

// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
    #define ANG_HELPER_DLL_IMPORT __declspec(dllimport)
    #define ANG_HELPER_DLL_EXPORT __declspec(dllexport)
    #define ANG_HELPER_DLL_LOCAL
#else
    #if __GNUC__ >= 4
        #define ANG_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
        #define ANG_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
        #define ANG_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
    #else
        #define ANG_HELPER_DLL_IMPORT
        #define ANG_HELPER_DLL_EXPORT
        #define ANG_HELPER_DLL_LOCAL
    #endif
#endif

// Now we use the generic helper definitions above to define ANG_API and ANG_LOCAL.
// ANG_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// ANG_LOCAL is used for non-api symbols.

#ifdef ANG_DLL // defined if ANG is compiled as a DLL
    #ifdef ANG_DLL_EXPORTS // defined if we are building the ANG DLL (instead of using it)
        #define ANG_API ANG_HELPER_DLL_EXPORT
    #else
        #define ANG_API ANG_HELPER_DLL_IMPORT
    #endif // ANG_DLL_EXPORTS
    #define ANG_LOCAL ANG_HELPER_DLL_LOCAL
#else // ANG_DLL is not defined: this means ANG is a static lib.
    #define ANG_API
    #define ANG_LOCAL
#endif // ANG_DLL
