/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 controllerUtilities.cpp

 Controller Utilities Implementation
 Created 12/24/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include "controllerUtilities.h"

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;
using namespace str1ker;

/*----------------------------------------------------------*\
| Implementation
\*----------------------------------------------------------*/

string controllerUtilities::getControllerName(const string& path)
{
    const char* lastSep = strrchr(path.c_str(), '/');
    if (lastSep == NULL) return path;

    return lastSep + 1;
}

string controllerUtilities::getParentName(const string& path)
{
    // Find controller name
    const char* parent = path.c_str() + path.length() - 1;
    if (*parent == '/') parent--;

    while (*parent != '/' && parent >= path)
        parent--;

    // Find controller parent name
    const char* parentEnd = parent--;

    while (*parent != '/' && parent >= path)
        parent--;

    if (parent == path) return string();

    // Copy controller parent name
    int parentLength = parentEnd - parent - 1;

    string parentName(parentLength, 0);
    strncpy(&parentName[0], parent + 1, parentLength);

    return parentName;
}

string controllerUtilities::getParentPath(const string& path)
{
    const char* parent = path.c_str() + path.length() - 1;
    if (*parent == '/') parent--;

    while (*parent != '/' && parent >= path)
        parent--;

    if (parent == path) return string();

    int length = parent - path.c_str();

    string parentPath(length + 1, 0);
    strncpy(&parentPath[0], path.c_str(), length);

    return parentPath;
}

string controllerUtilities::getControllerPath(const string& path, const string& parentPath)
{
    string typePath = "/" + parentPath + "/";

    const char* typeNode = strstr(path.c_str(), typePath.c_str());
    if (typeNode == NULL) return string();

    // Path must end with /controller
    const char* leaf = path.c_str() + path.length() - 1;

    while (*leaf != '/' && leaf >= path)
        leaf--;

    if (strncmp(leaf, "/controller", strlen("/controller")) == 0)
    {
        int length = leaf - path.c_str();
        string controllerPath(length, '\0');

        strncpy(&controllerPath[0], path.c_str(), length);

        return controllerPath;
    }

    return string();
}