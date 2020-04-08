#pragma once

// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
  #define JAIL_HELPER_DLL_IMPORT __declspec(dllimport)
  #define JAIL_HELPER_DLL_EXPORT __declspec(dllexport)
  #define JAIL_HELPER_DLL_LOCAL
#else
  #if __GNUC__ >= 4
    #define JAIL_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
    #define JAIL_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
    #define JAIL_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define JAIL_HELPER_DLL_IMPORT
    #define JAIL_HELPER_DLL_EXPORT
    #define JAIL_HELPER_DLL_LOCAL
  #endif
#endif

// Now we use the generic helper definitions above to define JAIL_API and JAIL_LOCAL.
// JAIL_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// JAIL_LOCAL is used for non-api symbols.

#ifdef JAIL_DLL // defined if JAIL is compiled as a DLL
  #ifdef JAIL_DLL_EXPORTS // defined if we are building the JAIL DLL (instead of using it)
    #define JAIL_API JAIL_HELPER_DLL_EXPORT
  #else
    #define JAIL_API JAIL_HELPER_DLL_IMPORT
  #endif // JAIL_DLL_EXPORTS
  #define JAIL_LOCAL JAIL_HELPER_DLL_LOCAL
#else // JAIL_DLL is not defined: this means JAIL is a static lib.
  #define JAIL_API
  #define JAIL_LOCAL
#endif // JAIL_DLL
