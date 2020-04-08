#pragma once

// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
#define ACFT_HELPER_DLL_IMPORT __declspec(dllimport)
    #define ACFT_HELPER_DLL_EXPORT __declspec(dllexport)
    #define ACFT_HELPER_DLL_LOCAL
#else
    #if __GNUC__ >= 4
        #define ACFT_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
        #define ACFT_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
        #define ACFT_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
    #else
        #define ACFT_HELPER_DLL_IMPORT
        #define ACFT_HELPER_DLL_EXPORT
        #define ACFT_HELPER_DLL_LOCAL
    #endif
#endif

// Now we use the generic helper definitions above to define ACFT_API and ACFT_LOCAL.
// ACFT_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// ACFT_LOCAL is used for non-api symbols.

#ifdef ACFT_DLL // defined if ACFT is compiled as a DLL
#ifdef ACFT_DLL_EXPORTS // defined if we are building the ACFT DLL (instead of using it)
        #define ACFT_API ACFT_HELPER_DLL_EXPORT
    #else
        #define ACFT_API ACFT_HELPER_DLL_IMPORT
    #endif // ACFT_DLL_EXPORTS
    #define ACFT_LOCAL ACFT_HELPER_DLL_LOCAL
#else // ACFT_DLL is not defined: this means ACFT is a static lib.
    #define ACFT_API
    #define ACFT_LOCAL
#endif // ACFT_DLL
