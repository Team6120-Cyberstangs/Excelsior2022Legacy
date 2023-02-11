// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/Timer.h>
#include <fmt/core.h>
#include <string.h>



void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoLow, kAutoLow);
  m_chooser.AddOption(kAutoHigh, kAutoHigh);
  m_chooser.AddOption(kAutoTwoLow, kAutoTwoLow);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  driveSelector.SetDefaultOption(s_arcadeDrive, s_arcadeDrive);
  driveSelector.AddOption(s_arcadeDrive, s_arcadeDrive);
  driveSelector.AddOption(s_tankDrive, s_tankDrive);
  frc::SmartDashboard::PutData("Drive Mode", &driveSelector);

  leftMotors.SetInverted(true);

  driveEncoder.ConfigSensorDirection(true);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  double driveEncoderPosition = driveEncoder.GetPosition();
  frc::SmartDashboard::PutNumber("Drive Encoder", driveEncoderPosition);
}
void Robot::AutonomousInit() {
  driveEncoder.SetPosition(0.0);
  m_autoSelected = m_chooser.GetSelected();
 // m_autoSelected = frc::SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoLow) {
    shooterMotor0.Set(ControlMode::PercentOutput, -.32);
    shooterMotor1.Set(ControlMode::PercentOutput, -.32);
    sleep(1);
    conveyorMotor.Set(-1);
    sleep(2);
    shooterMotor0.Set(ControlMode::PercentOutput, 0);
    shooterMotor1.Set(ControlMode::PercentOutput, 0);
    conveyorMotor.Set(0);
    autoTimer.Reset();
    autoTimer.Start();
    units::second_t twoSeconds{2.0};
    while (autoTimer.Get() < twoSeconds ){
      tankDrive.TankDrive(.65, .65);;
    }
    tankDrive.TankDrive(0, 0);
    autoTimer.Stop();
  } 

  else if (m_autoSelected == kAutoHigh) {

    autoTimer.Reset();
    autoTimer.Start();
    
    units::second_t twoSeconds{2.1};
    while ( autoTimer.Get() < twoSeconds){
      tankDrive.TankDrive(.65, .65);
    }
    tankDrive.TankDrive(0, 0);

    shooterMotor0.Set(ControlMode::PercentOutput, -.60);
    shooterMotor1.Set(ControlMode::PercentOutput, -.60);
    sleep(2);
    conveyorMotor.Set(-1);
    sleep(2);
    shooterMotor0.Set(ControlMode::PercentOutput, 0);
    shooterMotor1.Set(ControlMode::PercentOutput, 0);
    conveyorMotor.Set(0);
  }

  else if (m_autoSelected == kAutoTwoLow){
    //Forward intake
    gyro.Reset();
    double desiredHeading = 180.0;
    double kP = .07;
    intakeMotor.Set(1);
    intakeLift.Set(frc::DoubleSolenoid::Value::kForward);
    units::second_t twoSeconds{2.4};
    units::second_t threeSeconds{4.0};
    double heading = 0.0;//gyro.GetAngle();
    double error = heading - gyro.GetAngle();
    autoTimer.Reset();
    autoTimer.Start();
    while (autoTimer.Get() < twoSeconds){
      tankDrive.TankDrive(-.5 + kP * error, -.5 - kP * error);
      error = heading - gyro.GetAngle();
      //printf("%lf ", error);
    }
    autoTimer.Stop();
    //pause
    intakeMotor.Set(1);
    sleep(1);

    intakeMotor.Set(0);
    intakeLift.Set(frc::DoubleSolenoid::Value::kReverse);
    sleep(1);

    //turn around
    error = desiredHeading - gyro.GetAngle();
    kP = .1;
    autoTimer.Reset();
    autoTimer.Start();
    while(error != 0 && autoTimer.Get() < threeSeconds){
      error = desiredHeading - gyro.GetAngle();
      tankDrive.TankDrive(kP * error, -kP * error);
    }
    autoTimer.Stop();
      
    //Ram on
    //leftDrive0.Set(ControlMode::PercentOutput, .3);
    //leftDrive1.Set(ControlMode::PercentOutput, .3);

    //rightDrive0.Set(ControlMode::PercentOutput, -.3);
    //rightDrive1.Set(ControlMode::PercentOutput, -.3);
    //sleep(5);

    //stop
    tankDrive.TankDrive(0, 0);

    //shoot
    shooterMotor0.Set(ControlMode::PercentOutput, -.70);
    shooterMotor1.Set(ControlMode::PercentOutput, -.70);
    //shooterMotor0.Set(ControlMode::PercentOutput, -.20);
    //shooterMotor1.Set(ControlMode::PercentOutput, -.20);
    sleep(1);
    conveyorMotor.Set(-1);
    sleep(1);
    conveyorMotor.Set(0);
    sleep(1);
    conveyorMotor.Set(-1);
    sleep(1);
    //Stop shoot, Drive back
    shooterMotor0.Set(ControlMode::PercentOutput, 0);
    shooterMotor1.Set(ControlMode::PercentOutput, 0);
    conveyorMotor.Set(0);
    //stop
    tankDrive.TankDrive(0, 0);
  }
  else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoLow) {
    // Custom Auto goes here
  } 
  else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  driveEncoder.SetPosition(0.0);
}

void Robot::TeleopPeriodic() {
  //Sets Up Left Drive
  //If statement creates a 10% deadband
  //leftDrive0.Set(ControlMode::ModeGoesHere, Controller.GetRawAxis(Button))
  //if( driverController.GetRawAxis(1) > .10 || driverController.GetRawAxis(1) < -.10 ){
  //  leftDrive0.Set(ControlMode::PercentOutput, -driverController.GetRawAxis(1));
  //  leftDrive1.Set(ControlMode::PercentOutput, -driverController.GetRawAxis(1));
  //}
  //else{
  //  leftDrive0.Set(ControlMode::PercentOutput, 0);
  //  leftDrive1.Set(ControlMode::PercentOutput, 0);
 // }

  driveSelected = driveSelector.GetSelected();
  //Set Arcade Drive
  if(driveSelected == s_arcadeDrive){
    tankDrive.SetDeadband(.1);
    double leftSide = driverController.GetRawAxis(1) - driverController.GetRawAxis(4)*0.75;
    double rightSide = driverController.GetRawAxis(1) + driverController.GetRawAxis(4)*0.75;
    if(abs(leftSide) > 1){
      rightSide = rightSide/abs(leftSide);
    }
    else if(abs(rightSide) > 1){
      leftSide = leftSide/abs(rightSide);
    }
    tankDrive.TankDrive(leftSide, rightSide);
  }
  else if(driveSelected == s_tankDrive){
    tankDrive.SetDeadband(.1);
    double leftSide = driverController.GetRawAxis(1);
    double rightSide = driverController.GetRawAxis(5);
    tankDrive.TankDrive(leftSide, rightSide);
  }
  
  //Intake Motor Logic
  //IF Statement - if Trigger is not pressed and eject is, then run motor in reverse
  //ELSE - Run the motor according to the trigger state
  if (driverController.GetRawAxis(2) == 0 && driverController.GetRawButton(5) == true){
    intakeMotor.Set(-1);
  }
  else{
    intakeMotor.Set(driverController.GetRawAxis(2));
  }


  //Intake Lift Logic
  //IF - Intake OR Eject are pressed, Bring Down Intake Lift
  //ELSE - Bring UP Intake Lift
  if( driverController.GetRawAxis(2) > .10 || driverController.GetRawButton(5) == true){
    intakeLift.Set(frc::DoubleSolenoid::Value::kForward);
  }
  else{
    intakeLift.Set(frc::DoubleSolenoid::Value::kReverse);
  }

  //Setup Conveyor Motor
  //IF - Trigger is pressed conveyor will move
  //ELSE - conveyor will not move
  if( gunnerController.GetRawAxis(3) > .10 || driverController.GetRawAxis(3) > .10){
    conveyorMotor.Set(-1);
  }
  else if (gunnerController.GetRawAxis(2) > .10){
    conveyorMotor.Set(1);
  }
  else{
    conveyorMotor.Set(0);
  }

  //Shooter Logic
  bool isTurbo = false;
  if(gunnerController.GetRawButton(4)){
    isTurbo = true;
    shooterMotor0.Set(ControlMode::PercentOutput, -.50);
    shooterMotor1.Set(ControlMode::PercentOutput, -.50);
  }
  else if( driverController.GetRawButton(7)){
    shooterMotor0.Set(ControlMode::PercentOutput, -.70);
    shooterMotor1.Set(ControlMode::PercentOutput, -.70);
  }
  else if( driverController.GetRawAxis(3) > .10 || gunnerController.GetRawButton(1)){
    shooterMotor0.Set(ControlMode::PercentOutput, -.32);
    shooterMotor1.Set(ControlMode::PercentOutput, -.32);
  }
  else{
    shooterMotor0.Set(ControlMode::PercentOutput, 0);
    shooterMotor1.Set(ControlMode::PercentOutput, 0);
  }

  //Climber Logic
  //Left Motor
  if( gunnerController.GetRawButton(6) == true){
    climber0.Set(1);
  }
  else if(gunnerController.GetRawButton(2) == true){
    climber0.Set(-1);
  }
  else{
    climber0.Set(0);
  }
  //Right Motor
  if( gunnerController.GetRawButton(5) == true){
    climber1.Set(1);
  }
  else if(gunnerController.GetRawButton(3) == true){
    climber1.Set(-1);
  }
  else{
    climber1.Set(0);
  }


  //Smartdashboard code
  frc::SmartDashboard::PutBoolean("Turbo State", isTurbo);

  bool isSpooled;
  if( isTurbo == true){
    isSpooled = (shooterMotor0.GetMotorOutputPercent() <= -.38 && shooterMotor1.GetMotorOutputPercent() <= -.38);
  }
  else{
    isSpooled = (shooterMotor0.GetMotorOutputPercent() <= -.28 && shooterMotor1.GetMotorOutputPercent() <= -.28);
  }

  frc::SmartDashboard::PutBoolean("Shooter Ready", isSpooled);

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}
#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
