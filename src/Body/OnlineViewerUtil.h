
#ifndef ONLINEVIEWERCLIENT_H
#define ONLINEVIEWERCLIENT_H

#pragma warning(disable:4996)

#include "config.h"
#include <string>
#include <sstream>
#include <iostream>

#include <cnoidCorba/ORBwrap.h>
#include <cnoidCorba/OpenHRPCommon.hh>
#include <cnoidCorba/OnlineViewer.hh>

namespace cnoid
{
    HRP_UTIL_EXPORT OpenHRP::OnlineViewer_var getOnlineViewer(int argc, char **argv);
    HRP_UTIL_EXPORT OpenHRP::OnlineViewer_var getOnlineViewer(CORBA_ORB_var orb);
};

#endif
