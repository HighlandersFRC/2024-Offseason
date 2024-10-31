// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaults;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterDefault extends Command {
  /** Creates a new ShooterDefault. */
  Shooter shooter;
  private double initTime;
  private boolean isZeroed = false;
  private int numTimesOverCurrentLimit = 0;
  public ShooterDefault(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.initTime = Timer.getFPGATimestamp();
    this.numTimesOverCurrentLimit = 0;
    this.isZeroed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooter.setShooterAngle(Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG);
    // if(shooter.getShooterAngle() < -60 && Math.abs(shooter.getShooterAngleTarget()-Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG) < 1) {
    //   shooter.setShooterAnglePercent(0.0);
    // }
    // shooter.setShooterPercent(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
