/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 controllerUtilities.h

 Controller Utilities
 Created 12/24/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <string>
#include <cstring>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

using namespace std;

/*----------------------------------------------------------*\
| Declarations
\*----------------------------------------------------------*/

namespace str1ker::controllerUtilities {
    string getControllerName(const string& path);
    string getParentName(const string& path);
    string getParentPath(const string& path);
    string getControllerPath(const string& path, const string& parentPath);
} // namespace str1ker::controllerUtilities
