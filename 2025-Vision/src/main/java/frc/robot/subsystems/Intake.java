// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorState;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX intakeMotor = new TalonFX(20, "rio");

  private final TorqueCurrentFOC torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0).withMaxAbsDutyCycle(0.0);

  public enum IntakeState {
    INTAKE,
    OUTAKE,
    DEFAULT,
  }

  private IntakeState wantedState = IntakeState.DEFAULT;
  private IntakeState systemState = IntakeState.DEFAULT;

  public void setIntakeTorque(double current, double maxPercent) {
    intakeMotor.setControl(torqueCurrentFOCRequest.withOutput(current).withMaxAbsDutyCycle(maxPercent));
  }

  public Intake() {
  }

  public void setIntakePercent(double percent) {
    intakeMotor.set(percent);
  }

  public double getIntakeRPS() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  private IntakeState handleStateTransition() {
    switch (wantedState) {
      case INTAKE:
        return IntakeState.INTAKE;
      case OUTAKE:
        return IntakeState.OUTAKE;
      default:
        return IntakeState.DEFAULT;
    }
  }

  @Override
  public void periodic() {
    IntakeState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }
    Logger.recordOutput("Intake State", systemState);
    // switch (systemState) {
    // case INTAKE:
    // if (Math.abs(getIntakeRPS()) > 10) {
    // setIntakeTorque(-20, 0.7);
    // } else {
    // setIntakePercent(-0.7);
    // }
    // break;
    // case OUTAKE:
    // if (Math.abs(getIntakeRPS()) > 10) {
    // setIntakeTorque(20, 0.7);
    // } else {
    // setIntakePercent(0.7);
    // }
    // break;
    // default:
    // setIntakeTorque(-10, 0.2);
    // }
  }
}
