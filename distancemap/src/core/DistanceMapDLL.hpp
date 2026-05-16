#pragma once

/**
 * @file DistanceMapDLL.hpp
 * @brief DLL import/export macro for Windows shared-library builds.
 * @details Defines DISTANCEMAP_API as dllexport when building the DLL
 * (DISTANCEMAP_EXPORTS defined), dllimport when consuming it, and empty on
 * non-Windows or when LOTT_STATIC is defined (static link).
 */

#if defined(_WIN32) && !defined(LOTT_STATIC)
#ifdef DISTANCEMAP_EXPORTS
#define DISTANCEMAP_API __declspec(dllexport)
#else
#define DISTANCEMAP_API __declspec(dllimport)
#endif
#else
#define DISTANCEMAP_API
#endif
