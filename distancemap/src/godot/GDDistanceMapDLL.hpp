#pragma once

#ifdef _WIN32
    #ifdef GDDISTANCEMAP_EXPORTS
        #define GDDISTANCE_MAP_API __declspec(dllexport)
    #else
        #define GDDISTANCE_MAP_API __declspec(dllimport)
    #endif
#else
    #define GDDISTANCE_MAP_API
#endif
