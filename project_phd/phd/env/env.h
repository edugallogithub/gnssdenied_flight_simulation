#pragma once

// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
#define ENV_HELPER_DLL_IMPORT __declspec(dllimport)
    #define ENV_HELPER_DLL_EXPORT __declspec(dllexport)
    #define ENV_HELPER_DLL_LOCAL
#else
    #if __GNUC__ >= 4
        #define ENV_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
        #define ENV_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
        #define ENV_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
    #else
        #define ENV_HELPER_DLL_IMPORT
        #define ENV_HELPER_DLL_EXPORT
        #define ENV_HELPER_DLL_LOCAL
    #endif
#endif

// Now we use the generic helper definitions above to define ENV_API and ENV_LOCAL.
// ENV_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// ENV_LOCAL is used for non-api symbols.

#ifdef ENV_DLL // defined if ENV is compiled as a DLL
#ifdef ENV_DLL_EXPORTS // defined if we are building the ENV DLL (instead of using it)
        #define ENV_API ENV_HELPER_DLL_EXPORT
    #else
        #define ENV_API ENV_HELPER_DLL_IMPORT
    #endif // ENV_DLL_EXPORTS
    #define ENV_LOCAL ENV_HELPER_DLL_LOCAL
#else // ENV_DLL is not defined: this means ENV is a static lib.
    #define ENV_API
    #define ENV_LOCAL
#endif // ENV_DLL
