// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.AngleShooter;
import frc.robot.commands.defaults.ShooterDefault;

public class Shooter extends SubsystemBase {
  private final TalonFX leftShooter = new TalonFX(Constants.CANInfo.SHOOTER_LEFT_MOTOR_ID, "Canivore");
  private final TalonFX rightShooter = new TalonFX(Constants.CANInfo.SHOOTER_RIGHT_MOTOR_ID, "Canivore");
  private final TalonFX shooterAngle = new TalonFX(Constants.CANInfo.SHOOTER_ANGLE_MOTOR_ID, "Canivore");

  private final TalonFXConfiguration flywheelConfiguration = new TalonFXConfiguration();
  private final TalonFXConfiguration angleConfiguration = new TalonFXConfiguration();
  private double armZero;
  
  private final PositionVoltage anglePositionMotionProfileRequest = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  // private final PositionTorqueCurrentFOC anglePositionMotionProfileRequest = new PositionTorqueCurrentFOC(0.0);
  // private final DynamicMotionMagicTorqueCurrentFOC anglePositionMotionProfileRequest = new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0, 0, 0, 0, false, false, false);
  private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false);
  private final TorqueCurrentFOC angleFalconCurrentRequest = new TorqueCurrentFOC(0, 0, 0, false, false, false);
  /** Creates a new Shooter. */
  public Shooter() {}

  public void init() {
    this.flywheelConfiguration.Slot0.kP = 6;
    this.flywheelConfiguration.Slot0.kI = 0;
    this.flywheelConfiguration.Slot0.kD = 0;
    this.flywheelConfiguration.Slot0.kS = 0;
    this.flywheelConfiguration.Slot0.kV = 0.2;
    
    // this.flywheelConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    // this.flywheelConfiguration.CurrentLimits.SupplyCurrentThreshold = 100;
    // this.flywheelConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    // this.flywheelConfiguration.CurrentLimits.StatorCurrentLimit = 100;
    
    // this.angleConfiguration.Feedback.SensorToMechanismRatio = Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO;
    this.angleConfiguration.Slot0.kP = 1.7;
    this.angleConfiguration.Slot0.kI = 0.00;
    this.angleConfiguration.Slot0.kD = 0.05;
    this.angleConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    this.angleConfiguration.CurrentLimits.SupplyCurrentThreshold = 60;
    this.angleConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    this.angleConfiguration.CurrentLimits.StatorCurrentLimit = 60;
    // this.angleConfiguration.Slot0.kS = 1;
    // this.angleConfiguration.Slot0.kV = 3;
    // this.angleConfiguration.Slot0.kG = -2;
    this.angleConfiguration.MotionMagic.MotionMagicAcceleration = 1;
    this.angleConfiguration.Feedback.RotorToSensorRatio = 1.0;
    // this.angleConfiguration.Feedback.SensorToMechanismRatio = Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO;
    this.angleConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    this.angleConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;


    leftShooter.getConfigurator().apply(flywheelConfiguration);
    rightShooter.getConfigurator().apply(flywheelConfiguration);
    shooterAngle.getConfigurator().apply(angleConfiguration);
    this.shooterAngle.setNeutralMode(NeutralModeValue.Brake);
    setShooterEncoder(Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG);
    System.out.println("constant: " + Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG);
    setDefaultCommand(new ShooterDefault(this));
  }

  public boolean alignedPreset = true;

  public void setShooterPercent(double left, double right){
    leftShooter.set(-left);
    rightShooter.set(-right);
  }

  public void setShooterAnglePercent(double percent) {
    shooterAngle.set(percent);
  }

  public double getShooterAngleTarget( ) {
    return (anglePositionMotionProfileRequest.Position*360/Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO);
  }

  public void setShooterAngle(double angle /* degrees */) {
    shooterAngle.getConfigurator().apply(angleConfiguration);
    this.shooterAngle.setControl(this.anglePositionMotionProfileRequest.withPosition(Constants.degreesToRotations(angle)*Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO));
    SmartDashboard.putString("goal", shooterAngle.getAppliedControl().toString());
    SmartDashboard.putNumber("closed loop error", shooterAngle.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber("diferential loop error", shooterAngle.getDifferentialClosedLoopError().getValueAsDouble());
  }

  public void setShooterEncoder(double degrees){
    System.out.println("degrees: " + degrees);
    double rotations = Constants.degreesToRotations(degrees);
    double val = rotations * Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO;
    // System.out.println("New Value: " + val);
    this.shooterAngle.setPosition(val);
  }

  public double getAngleCurrent(){
    return this.shooterAngle.getStatorCurrent().getValueAsDouble();
  }

  public void setAngleTorqueCurrent(double current, double maxPercent){
    this.shooterAngle.setControl(this.angleFalconCurrentRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  public double getShooterAngle() {
    // SmartDashboard.putString("Angle Request", this.shooterAngle.getAppliedControl().getControlInfo().toString());
    // SmartDashboard.putNumber("zero ", (shooterAngle.getRotorPosition().getValueAsDouble()*360)/Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO);
    return (((shooterAngle.getPosition().getValueAsDouble()/Constants.Ratios.SHOOTER_ANGLE_GEAR_RATIO))*360);
  }

  public double getRawShooterAngle() {
    return (((shooterAngle.getRotorPosition().getValueAsDouble())*360));
  }

  public void setShooterRPM(double left, double right){
    this.leftShooter.setControl(this.flywheelVelocityRequest.withVelocity(Constants.RPMToRPS(left) * Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO));
    this.rightShooter.setControl(this.flywheelVelocityRequest
        .withVelocity(Constants.RPMToRPS(-right) * Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO));
  }

  public double getLeftShooterRPM() {
    return (Constants.RPSToRPM(leftShooter.getRotorVelocity().getValueAsDouble())) / Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO;
  }

  public double getRightShooterRPM() {
    return (Constants.RPSToRPM(rightShooter.getRotorVelocity().getValueAsDouble())) / Constants.Ratios.SHOOTER_FLYWHEEL_GEAR_RATIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Angle Motor Current", shooterAngle.getTorqueCurrent().getValue());
    Logger.recordOutput("shooter angle", shooterAngle.getPosition().getValueAsDouble());
  }
}
