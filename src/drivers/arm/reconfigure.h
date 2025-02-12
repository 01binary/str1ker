
/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 reconfigure.h
 ROS Dynamic Reconfigure Interface
 Copyright (C) 2025 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

#pragma once

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <ArduinoSTL.h>
#include <map>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

extern const String NAMESPACE;

const int RECONFIGURE_DELAY = 1000;
const String DESCRIPTIONS = NAMESPACE + "parameter_descriptions";
const String UPDATES = NAMESPACE + "parameter_updates";
const String SET = NAMESPACE + "set_parameters";

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef dynamic_reconfigure::Reconfigure::Request ReconfigureReq;
typedef dynamic_reconfigure::Reconfigure::Response ReconfigureRes;
typedef ros::ServiceServer<ReconfigureReq, ReconfigureRes> ReconfigureSrv;

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

template<typename T> struct Setting
{
  T* value;
  String description;
  T min;
  T max;

  Setting(): value(nullptr), min(0), max(0)
  {
  }

  Setting(T* address, T minValue, T maxValue, const char* desc):
    value(address),
    min(minValue),
    max(maxValue)
  {
    if (desc)
    {
      description = desc;
    }
  }
};

struct Group
{
  std::map<String, Setting<int>> ints;
  std::map<String, Setting<bool>> bools;
  std::map<String, Setting<double>> doubles;

  Group& describe(const char* name, int* address, int minValue, int maxValue, const char* description)
  {
    ints[name] = Setting<int>(address, minValue, maxValue, description);
    return *this;
  }

  Group& describe(const char* name, double* address, double minValue, double maxValue, const char* description)
  {
    doubles[name] = Setting<double>(address, minValue, maxValue, description);
    return *this;
  }

  Group& describe(const char* name, bool* address, const char* description)
  {
    bools[name] = Setting<bool>(address, false, true, description);
    return *this;
  }
};

class Reconfigure
{
public:
  std::map<String, Group> groups;

public:
  Group& describe(const char* name)
  {
    if (groups.find(name) == groups.end())
    {
      groups[name] = Group();
    }

    return groups[name];
  }

  void write(dynamic_reconfigure::ConfigDescription& description)
  {
    // Describe all settings
  }

  void write(dynamic_reconfigure::Config& config)
  {
    // Report values for all settings
  }

  void write(ReconfigureRes& res)
  {
    // Set values for all settings
  }

  void read(const ReconfigureReq& req)
  {
    write(req.config);
  }
};

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

extern ros::NodeHandle node;
void configureSettings(const ReconfigureReq &req, ReconfigureRes& res);

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

Reconfigure settings;

dynamic_reconfigure::ConfigDescription configDescription;
ros::Publisher descriptionPublisher(DESCRIPTIONS.c_str(), &configDescription);

dynamic_reconfigure::Config configUpdate;
ros::Publisher updatePublisher(UPDATES.c_str(), &configUpdate);

ReconfigureSrv reconfigureServer(SET.c_str(), &configureSettings);

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

void initializeDynamicReconfigure()
{
  node.advertise(descriptionPublisher);
  node.advertise(updatePublisher);
  node.advertiseService(reconfigureServer);
  delay(RECONFIGURE_DELAY);
}

void advertiseSettings()
{
  settings.write(configDescription);
  descriptionPublisher.publish(&configDescription);
}

void advertiseSettingValues()
{
  settings.write(configUpdate);
  updatePublisher.publish(&configUpdate);
}

void configureSettings(const ReconfigureReq &req, ReconfigureRes& res)
{
  settings.read(req);
  settings.write(res);
}
