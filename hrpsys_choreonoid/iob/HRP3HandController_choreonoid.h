// -*- C++ -*-
/*!
 * @file  HRP3HandController.h
 * @brief hand control based on jsk usb driver
 * @date  $Date$
 *
 * $Id$
 */

#ifndef HRP3HANDCONTROLLER_CHOREONOID_H
#define  HRP3HANDCONTROLLER_CHOREONOID_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include "HRPDataTypes.hh"

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "HRP3HandControllerService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

#include "HRP3HandController.h"
#include "handcontrol.h"

class HRP3HandController_choreonoid
  : public HRP3HandController
{
 public:
  HRP3HandController_choreonoid(RTC::Manager* manager);

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};


extern "C"
{
  void HRP3HandController_choreonoidInit(RTC::Manager* manager);
};

#endif // HRP3HANDCONTROLLER_CHOREONOID_H
