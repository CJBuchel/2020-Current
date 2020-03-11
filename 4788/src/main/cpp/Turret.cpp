#include "Turret.h"
#include <iostream>

using namespace wml;
using namespace wml::controllers;

Turret::Turret(Gearbox &Rotation,
 							 Gearbox &VerticalAxis, 
							 Gearbox &FlyWheel, 
							 sensors::BinarySensor &LeftLimit, 
							 sensors::BinarySensor &AngleDownLimit, 
							 SmartControllerGroup &contGroup, 
							 std::shared_ptr<nt::NetworkTable> &visionTable,
							 std::shared_ptr<nt::NetworkTable> &rotationTable,
							 bool &FlyWheelToggle, 
							 bool &TurretToggle,
							 int &autoSelector,
							 bool &StartDoComplete,
							 bool &strt,
							 bool &p1,
							 bool &p2,
							 bool &p3,
							 bool &end): 
							 
							 _RotationalAxis(Rotation),
						   _VerticalAxis(VerticalAxis), 
							 _FlyWheel(FlyWheel), 
							 _LeftLimit(LeftLimit), 
							 _AngleDownLimit(AngleDownLimit), 
							 _contGroup(contGroup), 
							 _visionTable(visionTable),
							 _rotationTable(rotationTable),
							 _FlyWheelToggle(FlyWheelToggle),
							 _TurretToggle(TurretToggle),
							 _autoSelector(autoSelector),
							 _StartDoComplete(StartDoComplete),
							 _strt(strt),
							 _p1(p1),
							 _p2(p2),
							 _p3(p3),
							 _end(end){
	table = _visionTable->GetSubTable("Target");
	table_2 =  _rotationTable->GetSubTable("turretRotation");

	imageHeight = table->GetNumber("ImageHeight", 0); 
	imageWidth = table->GetNumber("ImageWidth", 0);
}


void Turret::_Update(double dt) {
	switch(_current_state) {
		case TurretState::MANUAL_AIM:
			// Manual Angle Control
			AngularPower = std::fabs(_contGroup.Get(ControlMap::TurretManualAngle)) > ControlMap::joyDeadzone ? -_contGroup.Get(ControlMap::TurretManualAngle) : 0;

			// Manual Rotation Control
			RotationPower = std::fabs(_contGroup.Get(ControlMap::TurretManualRotate)) > ControlMap::joyDeadzone ? _contGroup.Get(ControlMap::TurretManualRotate) : 0;
			RotationPower = (fabs(RotationPower) * RotationPower);
		break;
		case TurretState::AUTO_AIM:
			if (!_TurretToggle) {
				if (targetX > imageWidth || targetY > imageHeight) {
					std::cout << "Error: Target is artifacting" << std::endl;
				} else {
					if (_FlyWheel.encoder->GetEncoderAngularVelocity() < 350)
						RotationPower = XAutoAimCalc(dt, targetX);
					AngularPower = YAutoAimCalc(dt, targetY);
				}
			} 
		break;
	}
	// Limits Turret Speed
	RotationPower *= ControlMap::MaxTurretSpeed; 
	AngularPower *= ControlMap::MaxTurretAngularSpeed;
}

void Turret::ZeroTurret() {
	double dt = 0;
	Turret::TurretZeroAngle();

	// Maxed Vertical Axis & Zero Flywheel
	MaxAngleRotations = (_VerticalAxis.encoder->GetEncoderRotations() + ControlMap::TurretEncoderSafeZone);
	_FlyWheel.encoder->ZeroEncoder();
}

void Turret::TeleopOnUpdate(double dt) {
	if (!TurretZeroed) {
		TurretZeroAngle();
	}

	targetX = table->GetNumber("Target_X", 0);
	targetY = table->GetNumber("Target_Y", 0);

	imageHeight = table->GetNumber("ImageHeight", 0); 
	imageWidth = table->GetNumber("ImageWidth", 0);

	// Tune Turret PID (If active)
	PIDTuner();

	// Switch Turret State
	if (_contGroup.Get(ControlMap::TurretAutoAimAxis) > ControlMap::triggerDeadzone) {
		_current_state = TurretState::AUTO_AIM;
	} else {
		_current_state = TurretState::MANUAL_AIM;
	}
	_Update(dt);

	if (_contGroup.Get(ControlMap::RevFlyWheel, Controller::ONRISE)) {
		bool GetFlyWheel = _FlyWheel.transmission->GetInverted();
		RevFlywheelEntry = table->GetEntry("RevFlywheel");
		RevFlywheelEntry.SetBoolean(GetFlyWheel);
		_FlyWheel.transmission->SetInverted(!GetFlyWheel);
	}

	if (!_FlyWheelToggle) {
		if (_contGroup.Get(ControlMap::TurretFlyWheelSpinUp) > ControlMap::triggerDeadzone) {
			FlyWheelAutoSpinup();
		} else {
			FlyWheelPower = 0;
		}
	}

	if (_FlyWheelToggle) {RotationPower = 0;}

	

	// Flywheel Feedback
	ContFlywheelFeedback();

	table_2->PutNumber("Turret_Min", MinRotation);
	table_2->PutNumber("Turret_Max", MaxRotation);



	_RotationalAxis.transmission->SetVoltage(12 * RotationPower);
	_VerticalAxis.transmission->SetVoltage(12 * AngularPower);
	_FlyWheel.transmission->SetVoltage(12 * FlyWheelPower);
}

void Turret::AutoOnUpdate(double dt) {}


// @TODO Turret Test
void Turret::TestOnUpdate(double dt) {

	_RotationalAxis.transmission->SetVoltage(12 * RotationPower);
	_VerticalAxis.transmission->SetVoltage(12 * AngularPower);
	_FlyWheel.transmission->SetVoltage(12 * FlyWheelPower);

}