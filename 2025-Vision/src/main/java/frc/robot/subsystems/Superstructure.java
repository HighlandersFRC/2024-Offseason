package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.DriveState;
import frc.robot.subsystems.Elevator.ElevatorState;

public class Superstructure extends SubsystemBase {
  private Drive drive;
  private Elevator elevator;

  public enum SuperState {
    CYCLING,
    IDLE,
    ELEVATOR_UP,
    ELEVATOR_MID,
  }

  private SuperState wantedSuperState = SuperState.IDLE;
  private SuperState currentSuperState = SuperState.IDLE;

  public Superstructure(Drive drive, Elevator elevator) {
    this.drive = drive;
    this.elevator = elevator;
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
        handleCYCLINGState();
        // Cycling state
        break;
      case IDLE:
        // Idle state
        handleIDLEState();
        break;
      case ELEVATOR_UP:
        // Idle state
        handleElevatorUPState();
        break;
      case ELEVATOR_MID:
        // Idle state
        handleElevatorMIDState();
        break;
      default:
        handleIDLEState();
        break;
    }
  }

  /**
   * This function handles the state transitions of the Superstructure subsystem.
   * It updates the current state based on the wanted state and performs necessary
   * actions.
   *
   * @return SuperState - The current state of the Superstructure subsystem after
   *         handling the state transitions.
   *
   * @param wantedSuperState The desired state of the Superstructure subsystem.
   *
   * @see SuperState
   */
  private SuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case CYCLING:
        // Cycling state
        currentSuperState = SuperState.CYCLING;
        break;
      case IDLE:
        // Idle state
        currentSuperState = SuperState.IDLE;
        break;
      case ELEVATOR_MID:
        // Mid state
        currentSuperState = SuperState.ELEVATOR_MID;
        break;
      case ELEVATOR_UP:
        // Up state
        currentSuperState = SuperState.ELEVATOR_UP;
        break;
      default:
        currentSuperState = SuperState.IDLE;
        break;
    }
    return currentSuperState;
  }

  /**
   * This function handles the CYCLING state of the Superstructure subsystem.
   * In the CYCLING state, the drive subsystem is set to its default state.
   *
   * @return void - This function does not return any value.
   */
  public void handleCYCLINGState() {
    drive.setWantedState(DriveState.DEFAULT);
    elevator.setWantedState(ElevatorState.IDLE);
  }

  /**
   * This function handles the IDLE state of the Superstructure subsystem.
   * In the IDLE state, the drive subsystem is set to its IDLE state.
   *
   * @return void - This function does not return any value.
   */
  public void handleIDLEState() {
    drive.setWantedState(DriveState.IDLE);
    elevator.setWantedState(ElevatorState.IDLE);
  }

  public void handleElevatorMIDState() {
    elevator.setWantedState(ElevatorState.MID);
  }

  public void handleElevatorUPState() {
    elevator.setWantedState(ElevatorState.UP);
  }

  @Override
  public void periodic() {
    currentSuperState = handleStateTransitions();
    Logger.recordOutput("Super State", currentSuperState);
    applyStates();
  }
}
