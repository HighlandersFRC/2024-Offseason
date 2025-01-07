// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Drive.DriveState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Superstructure.SuperState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L2AutoPlace extends SequentialCommandGroup {
  /** Creates a new autoL2Place. */
  
  public L2AutoPlace(Superstructure superstructure, Elevator elevator, Drive drive, Intake intake, Peripherals peripherals) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetElevatorState(superstructure, elevator, SuperState.ELEVATOR_L2, true),
      new MoveToPoint(drive, drive.getReefClosestSetpoint(drive.getMT2Odometry())[0], drive.getReefClosestSetpoint(drive.getMT2Odometry())[1], drive.getReefClosestSetpoint(drive.getMT2Odometry())[2], true),
      new ParallelRaceGroup(
      new SetDriveThetaSetpoint(drive, peripherals),
      new WaitCommand(0.1)
      ),
      new ParallelRaceGroup(
      new WaitCommand(0.2),
      new SetElevatorState(superstructure, elevator, SuperState.ELEVATOR_MID, false)
      ),
      new ParallelRaceGroup(
        new RunOutake(intake),
        new WaitCommand(0.1)
      )
      

      );
  }
}
