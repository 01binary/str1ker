/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 params.h
 Shared parameter-loading helpers
 Copyright (C) 2025 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ros.h>
#include <stdio.h>

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

inline void makeGroupPath(
  char* path,
  size_t pathSize,
  const char* group,
  const char* child)
{
  const bool hasGroup = group != nullptr && group[0] != '\0';
  const bool hasChild = child != nullptr && child[0] != '\0';

  if (hasGroup && hasChild)
  {
    snprintf(path, pathSize, "%s/%s", group, child);
  }
  else if (hasGroup)
  {
    snprintf(path, pathSize, "%s", group);
  }
  else if (hasChild)
  {
    snprintf(path, pathSize, "%s", child);
  }
  else
  {
    path[0] = '\0';
  }
}

template<typename T>
inline void loadParam(
  ros::NodeHandle& node,
  const char* group,
  const char* key,
  T& value)
{
  char path[96] = {0};
  if (group != nullptr && group[0] != '\0')
  {
    snprintf(path, sizeof(path), "~%s/%s", group, key);
  }
  else
  {
    snprintf(path, sizeof(path), "~%s", key);
  }

  node.getParam(path, &value);
}

inline void loadBoolParam(
  ros::NodeHandle& node,
  const char* group,
  const char* key,
  bool& value)
{
  int value_i = value ? 1 : 0;
  loadParam(node, group, key, value_i);
  value = value_i != 0;
}
