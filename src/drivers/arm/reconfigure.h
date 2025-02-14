
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
#include <EEPROMex.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>

/*----------------------------------------------------------*\
| Constants
\*----------------------------------------------------------*/

extern const String NAMESPACE;

const String DESCRIPTIONS = NAMESPACE + "parameter_descriptions";
const String UPDATES = NAMESPACE + "parameter_updates";
const String SET = NAMESPACE + "set_parameters";
const int MAX_WRITES = 50;

/*----------------------------------------------------------*\
| Definitions
\*----------------------------------------------------------*/

typedef dynamic_reconfigure::Reconfigure::Request ReconfigureReq;
typedef dynamic_reconfigure::Reconfigure::Response ReconfigureRes;
typedef ros::ServiceServer<ReconfigureReq, ReconfigureRes> ReconfigureSrv;

/*----------------------------------------------------------*\
| Classes
\*----------------------------------------------------------*/

template<typename T> struct ConfigurationSetting
{
  T* value;
  String description;
  T min;
  T max;
  T deflt;

  ConfigurationSetting(): value(nullptr), min(0), max(0), deflt(0)
  {
  }

  ConfigurationSetting(T* address, T minValue, T maxValue, T defaultValue, const char* desc):
    value(address),
    min(minValue),
    max(maxValue),
    deflt(defaultValue),
    description(desc)
  {
  }
};

typedef std::map<String, ConfigurationSetting<int>> IntConfigMap;
typedef std::map<String, ConfigurationSetting<bool>> BoolConfigMap;
typedef std::map<String, ConfigurationSetting<double>> DoubleConfigMap;

struct ConfigurationGroup
{
  IntConfigMap ints;
  BoolConfigMap bools;
  DoubleConfigMap doubles;

  ConfigurationGroup& registerSetting(
    const char* name, int* address, int minValue, int maxValue, int defaultValue, const char* description)
  {
    ints[name] = ConfigurationSetting<int>(
      address, minValue, maxValue, defaultValue, description);

    return *this;
  }

  ConfigurationGroup& registerSetting(
    const char* name, double* address, double minValue, double maxValue, double defaultValue, const char* description)
  {
    doubles[name] = ConfigurationSetting<double>(
      address, minValue, maxValue, defaultValue, description);

    return *this;
  }

  ConfigurationGroup& registerSetting(
    const char* name, bool* address, bool defaultValue, const char* description)
  {
    bools[name] = ConfigurationSetting<bool>(
      address, false, true, defaultValue, description);

    return *this;
  }

  void load()
  {
    for (IntConfigMap::iterator pos = ints.begin();
        pos != ints.end();
        pos++)
    {
      *pos->second.value = EEPROM.readInt(EEPROM.getAddress(sizeof(int)));
    }

    for (BoolConfigMap::iterator pos = bools.begin();
        pos != bools.end();
        pos++)
    {
      *pos->second.value = EEPROM.readInt(EEPROM.getAddress(sizeof(int)));
    }

    for (DoubleConfigMap::iterator pos = doubles.begin();
        pos != doubles.end();
        pos++)
    {
      *pos->second.value = EEPROM.readDouble(EEPROM.getAddress(sizeof(double)));
    }
  }

  void save()
  {
    // Write int settings
    for (IntConfigMap::iterator pos = ints.begin();
        pos != ints.end();
        pos++)
    {
      EEPROM.writeInt(EEPROM.getAddress(sizeof(int)), *pos->second.value);
    }

    for (BoolConfigMap::iterator pos = bools.begin();
        pos != bools.end();
        pos++)
    {
      EEPROM.writeInt(EEPROM.getAddress(sizeof(int)), *pos->second.value);
    }

    for (DoubleConfigMap::iterator pos = doubles.begin();
        pos != doubles.end();
        pos++)
    {
      EEPROM.writeDouble(EEPROM.getAddress(sizeof(double)), *pos->second.value);
    }
  }
};

class Reconfigure
{
public:
  std::map<String, ConfigurationGroup> groups;

public:
  ConfigurationGroup& group(const char* name)
  {
    if (groups.find(name) == groups.end())
    {
      groups[name] = ConfigurationGroup();
    }

    return groups[name];
  }

  void describeConfiguration(dynamic_reconfigure::ConfigDescription& description)
  {
    //description.dflt
  }

  void getConfiguration(dynamic_reconfigure::Config& config)
  {
  }

  void setConfiguration(const ReconfigureReq& req)
  {
    for (std::map<String, ConfigurationGroup>::iterator pos = groups.begin();
        pos != groups.end();
        pos++)
    {
      // Update settings
      // Write settings to flash
    }
  }

  void loadSettings()
  {
    // Initialize flash storage
    EEPROM.setMemPool(0, EEPROMSizeMega);
    EEPROM.setMaxAllowedWrites(MAX_WRITES);

    while(!EEPROM.isReady())
    {
      delay(100);
    }

    // Read signature
    int signature = EEPROM.readInt(EEPROM.getAddress(sizeof(int)));

    if (signature == 0xFF)
    {
      // Settings have never been saved
      return;
    }

    // Read settings
    for (std::map<String, ConfigurationGroup>::iterator pos = groups.begin();
        pos != groups.end();
        pos++)
    {
      pos->second.load();
    }
  }

  void saveSettings()
  {
    // Write signature
    EEPROM.writeInt(EEPROM.getAddress(sizeof(int)), 0);

    // Write settings
    for (std::map<String, ConfigurationGroup>::iterator pos = groups.begin();
        pos != groups.end();
        pos++)
    {
      pos->second.save();
    }
  }
};

/*----------------------------------------------------------*\
| Forward Declarations
\*----------------------------------------------------------*/

extern ros::NodeHandle node;
void configure(const ReconfigureReq &req, ReconfigureRes& res);

/*----------------------------------------------------------*\
| Variables
\*----------------------------------------------------------*/

Reconfigure config;

dynamic_reconfigure::ConfigDescription configDescription;
ros::Publisher descriptionPublisher(DESCRIPTIONS.c_str(), &configDescription);

dynamic_reconfigure::Config configUpdate;
ros::Publisher updatePublisher(UPDATES.c_str(), &configUpdate);

ReconfigureSrv reconfigureServer(SET.c_str(), &configure);

/*----------------------------------------------------------*\
| Functions
\*----------------------------------------------------------*/

Reconfigure& initializeReconfigure()
{
  node.advertise(descriptionPublisher);
  node.advertise(updatePublisher);
  node.advertiseService(reconfigureServer);

  return config;
}

void advertiseConfiguration()
{
  config.describeConfiguration(configDescription);
  descriptionPublisher.publish(&configDescription);
}

void advertiseConfigurationValues()
{
  config.getConfiguration(configUpdate);
  updatePublisher.publish(&configUpdate);
}

void configure(const ReconfigureReq &req, ReconfigureRes& res)
{
  config.setConfiguration(req);
  config.getConfiguration(res.config);
}
