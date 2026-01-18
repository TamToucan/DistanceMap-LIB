#pragma once

#ifdef _WIN32
#ifdef DISTANCEMAP_EXPORTS
#define DISTANCEMAP_API __declspec(dllexport)
#else
#define DISTANCEMAP_API __declspec(dllimport)
#endif
#else
#define DISTANCEMAP_API
#endif
