// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.defaults.ClimberDefault;

public class Climber extends SubsystemBase {
  Servo upServo = new Servo(2);
  Servo downServo = new Servo(3);
  /** Creates a new Climber. */
  public Climber() {
    setDefaultCommand(new ClimberDefault(this));
  }

  public double getUpServoAngle(){
    return upServo.getAngle();
  }

  public double getDownServoAngle(){
    return downServo.getAngle();
  }

  public void setUpServoAngle(double degrees){
    upServo.setAngle(degrees);
  }

  public void setDownServoAngle(double degrees){
    downServo.setAngle(degrees);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
