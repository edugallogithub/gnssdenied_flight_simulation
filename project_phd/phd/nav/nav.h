#pragma once

// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
#define NAV_HELPER_DLL_IMPORT __declspec(dllimport)
    #define NAV_HELPER_DLL_EXPORT __declspec(dllexport)
    #define NAV_HELPER_DLL_LOCAL
#else
    #if __GNUC__ >= 4
        #define NAV_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
        #define NAV_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
        #define NAV_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
    #else
        #define NAV_HELPER_DLL_IMPORT
        #define NAV_HELPER_DLL_EXPORT
        #define NAV_HELPER_DLL_LOCAL
    #endif
#endif

// Now we use the generic helper definitions above to define NAV_API and NAV_LOCAL.
// NAV_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// NAV_LOCAL is used for non-api symbols.

#ifdef NAV_DLL // defined if NAV is compiled as a DLL
#ifdef NAV_DLL_EXPORTS // defined if we are building the NAV DLL (instead of using it)
        #define NAV_API NAV_HELPER_DLL_EXPORT
    #else
        #define NAV_API NAV_HELPER_DLL_IMPORT
    #endif // NAV_DLL_EXPORTS
    #define NAV_LOCAL NAV_HELPER_DLL_LOCAL
#else // NAV_DLL is not defined: this means NAV is a static lib.
    #define NAV_API
    #define NAV_LOCAL
#endif // NAV_DLL
