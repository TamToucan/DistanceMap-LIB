#pragma once

#if defined(_WIN32) && !defined(LOTT_STATIC)
#ifdef DISTANCEMAP_EXPORTS
#define DISTANCEMAP_API __declspec(dllexport)
#else
#define DISTANCEMAP_API __declspec(dllimport)
#endif
#else
#define DISTANCEMAP_API
#endif
