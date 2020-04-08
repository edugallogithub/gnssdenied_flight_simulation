#pragma once

// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
    #define MATH_HELPER_DLL_IMPORT __declspec(dllimport)
    #define MATH_HELPER_DLL_EXPORT __declspec(dllexport)
    #define MATH_HELPER_DLL_LOCAL
#else
    #if __GNUC__ >= 4
        #define MATH_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
        #define MATH_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
        #define MATH_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
    #else
        #define MATH_HELPER_DLL_IMPORT
        #define MATH_HELPER_DLL_EXPORT
        #define MATH_HELPER_DLL_LOCAL
    #endif
#endif

// Now we use the generic helper definitions above to define MATH_API and MATH_LOCAL.
// MATH_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// MATH_LOCAL is used for non-api symbols.

#ifdef MATH_DLL // defined if MATH is compiled as a DLL
    #ifdef MATH_DLL_EXPORTS // defined if we are building the MATH DLL (instead of using it)
        #define MATH_API MATH_HELPER_DLL_EXPORT
    #else
        #define MATH_API MATH_HELPER_DLL_IMPORT
    #endif // MATH_DLL_EXPORTS
    #define MATH_LOCAL MATH_HELPER_DLL_LOCAL
#else // MATH_DLL is not defined: this means MATH is a static lib.
    #define MATH_API
    #define MATH_LOCAL
#endif // MATH_DLL
