// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/encoder.h>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/Joystick.h>

#include <frc/DoubleSolenoid.h>

#include <ctre/Phoenix/sensors/PigeonIMU.h>

#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/sensors/WPI_PigeonIMU.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/Timer.h>



//Includes CTRE support, used for VictorSPX Motor Controllers
#include <ctre/Phoenix.h>

//Includes Rev Library Support, used for Spark Max Motor Controllers
#include "rev/CANSparkMax.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoLow = "Low Shot";
  const std::string kAutoHigh = "High Shot";
  const std::string kAutoTwoLow = "Double Low Shot";
  std::string m_autoSelected;

  frc::SendableChooser<std::string> driveSelector;
  const std::string s_arcadeDrive = "Arcade Drive";
  const std::string s_tankDrive = "Tank Drive";
  std::string driveSelected;

  frc::Timer timer();

  //Setup joysticks 
  frc::Joystick driverController{0};
  frc::Joystick gunnerController{1};


  //Left Drive CAN ID's
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX leftDrive0 = {10};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX leftDrive1 = {1};

  //Right Drive CAN ID's
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX rightDrive0 = {2};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX rightDrive1 = {3};

  //Set Tank Drive
  frc::MotorControllerGroup leftMotors{leftDrive0, leftDrive1};
  frc::MotorControllerGroup rightMotors{rightDrive0, rightDrive1};

  frc::DifferentialDrive tankDrive{leftMotors, rightMotors};


  //intake motor CAN ID's
  rev::CANSparkMax intakeMotor{4, rev::CANSparkMax::MotorType::kBrushless};

  //Conveyor Motor CAN ID
  rev::CANSparkMax conveyorMotor{5, rev::CANSparkMax::MotorType::kBrushless};

  //Shooter Motor CAN ID's
  VictorSPX shooterMotor0 = {6};
  VictorSPX shooterMotor1 = {7};

  //Climber Motor CAN ID's
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX climber0 = {8};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX climber1 = {9};


  //Set Up Intake Solenoid
  frc::DoubleSolenoid intakeLift{frc::PneumaticsModuleType::CTREPCM, 0, 1};

  //Encoder
  ctre::phoenix::sensors::CANCoder driveEncoder{12};

  //Pigeon IMU
  ctre::phoenix::sensors::WPI_PigeonIMU gyro{11};

  frc::Timer autoTimer;
};
