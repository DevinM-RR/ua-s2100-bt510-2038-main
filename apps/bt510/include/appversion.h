
#ifndef _VERSION_H_
#define _VERSION_H_

#define APP_NAME_STRING "Regal Beloit Sensor Device"
//changes to APP_VERSION_STRING should also apply to "app_version" in settings.json
#define APP_VERSION_MAJOR 0
#define APP_VERSION_MINOR 0
#define APP_VERSION_BUILD 7
#define APP_VERSION_STRING                                                     \
	STRINGIFY(APP_VERSION_MAJOR)                                           \
	"." STRINGIFY(APP_VERSION_MINOR) "." STRINGIFY(APP_VERSION_BUILD)

#endif