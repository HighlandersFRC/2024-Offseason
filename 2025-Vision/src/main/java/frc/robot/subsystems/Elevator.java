package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final TalonFX elevatorMotor1 = new TalonFX(9, new CANBus(Constants.CANInfo.CANBUS_NAME));
  private final TalonFX elevatorMotor2 = new TalonFX(10, new CANBus(Constants.CANInfo.CANBUS_NAME));

  public enum ElevatorState {
    IDLE,
    UP,
    MID,
    DOWN,
  }

  private ElevatorState wantedState = ElevatorState.IDLE;
  private ElevatorState systemState = ElevatorState.IDLE;

  public Elevator() {
  }

  public void moveWithPercent(double percent) {
    elevatorMotor1.set(percent);
    elevatorMotor2.set(-percent);
  }

  public void moveElevatorToPosition(Constants.SetPoints.ElevatorPosition position) {

  }

  public void setWantedState(ElevatorState wantedState) {
    this.wantedState = wantedState;
  }

  private ElevatorState handleStateTransition() {
    switch (wantedState) {
      case IDLE:
        return ElevatorState.IDLE;
      case UP:
        return ElevatorState.UP;
      case MID:
        return ElevatorState.MID;
      case DOWN:
        return ElevatorState.DOWN;
      default:
        return ElevatorState.IDLE;
    }
  }

  @Override
  public void periodic() {
    ElevatorState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }
    Logger.recordOutput("Elevator State", systemState);
    switch (systemState) {
      case UP:
        moveElevatorToPosition(Constants.SetPoints.ElevatorPosition.kUP);
        break;
      case MID:
        moveElevatorToPosition(Constants.SetPoints.ElevatorPosition.kMID);
        break;
      case DOWN:
        moveElevatorToPosition(Constants.SetPoints.ElevatorPosition.kDOWN);
        break;
      case IDLE:
        moveWithPercent(0.0);
        break;
      default:
        moveWithPercent(0.0);
    }
  }
}
