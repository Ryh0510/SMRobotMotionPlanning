#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
#  ifdef POSTINTERFACE_SHAREDLIB_EXPORTS
#    define POSTINTERFACE_SHAREDLIB __declspec(dllexport)
#  else
#    define POSTINTERFACE_SHAREDLIB __declspec(dllimport)
#  endif
#else
#  define POSTINTERFACE_SHAREDLIB
#endif

#ifdef __cplusplus
}
#endif