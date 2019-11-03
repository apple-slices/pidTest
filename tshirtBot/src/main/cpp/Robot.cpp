/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <frc/AnalogInput.h>
#include "Robot.h"
#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <iostream>
#include <frc/Spark.h>
#include <frc/smartdashboard/SmartDashboard.h>


  rev::CANSparkMax m_leftLeadMotor{12, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightLeadMotor{14, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_leftFollowMotor{13, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightFollowMotor{15, rev::CANSparkMax::MotorType::kBrushless};
  frc::Spark m_cannonTriger(2);
  frc::Spark m_compressor(1);
  frc::AnalogInput m_psig(0);

  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  frc::Joystick m_stick{0};

  
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    
    m_leftLeadMotor.GetEncoder();
    m_rightLeadMotor.GetEncoder();
    m_leftFollowMotor.GetEncoder();
    m_rightFollowMotor.GetEncoder();


  m_cannonTriger.StopMotor();
  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);


}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
    
    if(m_stick.GetRawButtonPressed(5)) 
      m_cannonTriger.Set(1.0);
    else if(m_stick.GetRawButtonReleased(5))
      m_cannonTriger.Set(0.0);

    if(m_stick.GetRawButtonPressed(6)) 
      m_compressor.Set(1);
    else if(m_stick.GetRawButtonReleased(6))
      m_compressor.Set(0.0);
    m_robotDrive.ArcadeDrive(m_stick.GetRawAxis(1), -1 * m_stick.GetRawAxis(2));

    frc::SmartDashboard::PutNumber(kPsigName,m_psig.GetValue()*.064-13);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
