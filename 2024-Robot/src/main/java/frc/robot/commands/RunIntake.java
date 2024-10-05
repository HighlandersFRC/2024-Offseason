// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RunIntake extends Command {
  Intake intake;
  Feeder feeder;
  double percent;
  private boolean noteIn;
  /** Creates a new RunIntake. */
  public RunIntake(Intake intake, Feeder feeder, double percent) {
    this.intake = intake;
    this.feeder = feeder;
    this.percent = percent;
    addRequirements(intake, feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noteIn = false;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!intake.getBeamBreak()) {
      noteIn = true;
    }
    intake.setPercent(percent);
    feeder.setPercent(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPercent(0.0);
    feeder.setPercent(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intake.getBeamBreak() && noteIn) {
      return true;
    }
    return false;
  }
}
