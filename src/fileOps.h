#ifndef __FILEOPS_H__
#define __FILEOPS_H__


#include <Preferences.h>
#include "main.h"

extern SettingsData setting;

void load_configFile(SettingsData* pSetting);
void write_configFile(SettingsData* pSetting);

#endif