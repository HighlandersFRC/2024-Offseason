package frc.robot.subsystems;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.tools.controlloops.PID;
import frc.robot.subsystems.Drive.DriveState;

public class Superstructure extends SubsystemBase {
  private Drive drive;
  private Peripherals peripherals;

  public enum SuperState {
    CYCLING,
  }

  private SuperState wantedSuperState = SuperState.CYCLING;
  private SuperState currentSuperState = SuperState.CYCLING;
  private SuperState previousSuperState;

  public Superstructure(Drive drive, Peripherals peripherals) {
    this.drive = drive;
    this.peripherals = peripherals;
  }

  public void setWantedState(SuperState wantedState) {
    this.wantedSuperState = wantedState;
  }

  public Command setWantedSuperStateCommand(SuperState wantedSuperState) {
    return new InstantCommand(() -> setWantedState(wantedSuperState));
  }

  private void applyStates() {
    switch (currentSuperState) {
      case CYCLING:
        handleCyclingState();
        // Cycling state
        break;
      default:
        handleCyclingState();
        break;
    }
  }

  private SuperState handleStateTransitions() {
    previousSuperState = currentSuperState;
    switch (wantedSuperState) {
      case CYCLING:
        // Cycling state
        currentSuperState = SuperState.CYCLING;
        break;
      default:
        currentSuperState = SuperState.CYCLING;
        break;
    }
    return currentSuperState;
  }

  public void handleCyclingState() {
    drive.setWantedState(DriveState.DEFAULT);
  }

  @Override
  public void periodic() {
    currentSuperState = handleStateTransitions();
    applyStates();
  }
}
