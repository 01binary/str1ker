/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 utilities.h

 Motor and Encoder Utilities
 Created 09/20/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <limits>
#include <stdlib.h>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker::utilities {

/*----------------------------------------------------------*\
| Helper functions
\*----------------------------------------------------------*/

inline bool isZero(double value)
{
  return abs(value) < std::numeric_limits<double>::epsilon();
}

inline double clampZero(double value, double min, double max)
{
  if (value < min && !isZero(value)) return min;
  if (value > max) return max;

  return value;
}

template <class T> inline double clamp(T value, T min, T max)
{
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

inline double map(double value, double min, double max, double targetMin, double targetMax)
{
  return (value - min) / (max - min) * (targetMax - targetMin) + targetMin;
}

inline double mapZero(double value, double min, double max, double targetMin, double targetMax)
{
  if (!isZero(value))
  {
    return (value - min) / (max - min) * (targetMax - targetMin) + targetMin;
  }
  else
  {
    return 0.0;
  }
}

inline bool isSameSign(double a, double b)
{
  return (a >= 0.0) == (b >= 0.0);
}

} // namespace str1ker