#pragma once

#include "controllers/Controllers.h"
#include "RobotMap.h"

class MagLoader {
  public:
    MagLoader(wml::Gearbox &MagazineMotors, wml::controllers::SmartControllerGroup &contGroup); // also need to add the limit switch & indexing math.

    void TeleopOnUpdate(double dt);
    void AutoOnUpdate(double dt);
    void TestOnUpdate(double dt);

  private:
    wml::Gearbox &_MagazineMotors;
    wml::controllers::SmartControllerGroup &_contGroup;
};