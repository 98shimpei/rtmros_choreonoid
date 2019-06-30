// -*- C++ -*-
/*!
 * @file  HRP3HandController.cpp
 * @brief hand control based on jsk usb driver
 * $Date$
 *
 * $Id$
 */

#include "handcontrol.h"
#include "HRP3HandController_choreonoid.h"
#include "RobotHardwareService.hh"

#define deg2rad(x)((x)*M_PI/180)

// Module specification
// <rtc-template block="module_spec">
static const char* hrp3handcontroller_choreonoid_spec[] =
  {
    "implementation_id", "HRP3HandController_choreonoid",
    "type_name",         "HRP3HandController_choreonoid",
    "description",       "hand control",
    //    "version",           HRPSYS_PACKAGE_VERSION,
    "version",           "1.0",
    "vendor",            "JSK (hrpsys_choreonoid)",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    "conf.default.debugLevel", "0",
    ""
  };
// </rtc-template>

extern RTC::DataFlowComponentBase *self_ptr;
extern void iob_update();
extern void iob_finish();

HRP3HandController_choreonoid::HRP3HandController_choreonoid(RTC::Manager* manager)
  : HRP3HandController(manager)
{
  m_servoState.data.length(1);
  m_servoState.data[0].length(1);
}


RTC::ReturnCode_t HRP3HandController_choreonoid::onInitialize()
{
  std::cerr << "[hc choreonoid initialize]" << std::endl;
  self_ptr = this;
  RTC::ReturnCode_t ret = HRP3HandController::onInitialize();
  return RTC::RTC_OK;
}



RTC::ReturnCode_t HRP3HandController_choreonoid::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[hc choreonoid activate]" << std::endl;
  iob_update();
  std::cerr << "[hc choreonoid activate iob_update]" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t HRP3HandController_choreonoid::onExecute(RTC::UniqueId ec_id)
{
  iob_update();
  RTC::ReturnCode_t ret = HRP3HandController::onExecute(ec_id);
  iob_finish();
  return ret;
}

extern "C"
{

  void HRP3HandController_choreonoidInit(RTC::Manager* manager)
  {
    RTC::Properties profile(hrp3handcontroller_choreonoid_spec);
    manager->registerFactory(profile,
                             RTC::Create<HRP3HandController_choreonoid>,
                             RTC::Delete<HRP3HandController_choreonoid>);
  }

};


