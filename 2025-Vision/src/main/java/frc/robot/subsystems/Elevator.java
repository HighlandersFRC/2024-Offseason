package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private final TalonFX elevatorMotorMaster = new TalonFX(Constants.CANInfo.MASTER_ELEVATOR_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));
  private final TalonFX elevatorMotorFollower = new TalonFX(Constants.CANInfo.FOLLOWER_ELEVATOR_MOTOR_ID,
      new CANBus(Constants.CANInfo.CANBUS_NAME));

  private final PositionTorqueCurrentFOC positionTorqueFOCRequest = new PositionTorqueCurrentFOC(0);

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

  public void init() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.Slot0.kP = 3.5;
    elevatorConfig.Slot0.kI = 0.0;
    elevatorConfig.Slot0.kD = 0.4;
    elevatorConfig.Slot0.kG = 0.5;
    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 60;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    // elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 

    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevatorMotorMaster.getConfigurator().apply(elevatorConfig);
    elevatorMotorFollower.getConfigurator().apply(elevatorConfig);
    elevatorMotorMaster.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotorFollower.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotorMaster.setPosition(0.0);
    elevatorMotorFollower.setPosition(0.0);
  }

  public void moveWithPercent(double percent) {
    elevatorMotorMaster.set(percent);
    elevatorMotorFollower.set(-percent);
  }

  public void moveElevatorToPosition(Constants.SetPoints.ElevatorPosition position) {
    elevatorMotorMaster
        .setControl(positionTorqueFOCRequest.withPosition(Constants.Ratios.elevatorMetersToRotations(position.meters)));
    elevatorMotorFollower
        .setControl(positionTorqueFOCRequest.withPosition(-Constants.Ratios.elevatorMetersToRotations(position.meters)));
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
    Logger.recordOutput("1 position", elevatorMotorMaster.getPosition().getValueAsDouble());
    Logger.recordOutput("2 position", elevatorMotorFollower.getPosition().getValueAsDouble());
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
