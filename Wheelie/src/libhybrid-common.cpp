#ifdef HAVE_CONFIG_H
#include "config.h"

const char *libhybrid_package_string()
{
    static const char package_string[] = PACKAGE_NAME " " PACKAGE_VERSION "-" GIT_VERSION " configured on " HOSTNAME " at " BUILD_DATE "\nConfigured with" CONFIGURE_ARGS;
    return package_string;
}

#else

const char *libhybrid_package_string()
{
    static const char package_string[] = "No config.h available";
    return package_string;
}
#endif
