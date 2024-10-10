// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class DipShot extends Command {
  Intake intake;
  Shooter shooter;
  Feeder feeder;
  double left;
  double right;
  double angle;
  double startTime;
  double shootTime;
  double distToSpeakerMeters;
  double x;
  boolean shot = false;
  Drive drive;
  /** Creates a new RunShooter. */
  public DipShot(Drive drive, Intake intake, Shooter shooter, Feeder feeder, double left, double right, double angle /* input in RPM */  /* degrees, resting position is -70 */) {
    this.intake = intake;
    this.shooter = shooter;
    this.feeder = feeder;
    this.left = left;
    this.right = right;
    this.angle = angle;
    this.drive = drive;
    addRequirements(this.intake, this.shooter, this.feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setPercent(1);
    startTime = Timer.getFPGATimestamp();
    shot = false;
    shooter.alignedPreset = true;
    if (drive.getFieldSide() == "red"){
      // System.out.println("-----------red------");
      x = Constants.Physical.FIELD_LENGTH;
    } else {
      // System.out.println("-----------blue------");
      x = Constants.Physical.SPEAKER_X;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Shooter Angle", shooter.getShooterAngle());
    shooter.setShooterAngle(angle);
    shooter.setShooterRPM(left, right);
    // if (Timer.getFPGATimestamp() - startTime > 2){
    //   feeder.setPercent(0.5);
    //   intake.setPercent(0.4);
    // }
    if((Math.abs(shooter.getLeftShooterRPM() - left)/left < 0.1 && Math.abs(shooter.getShooterAngle()-angle) < 4 && Math.abs(shooter.getRightShooterRPM() + right)/right < 0.1/*   && shooter.alignedPreset */) || Timer.getFPGATimestamp() - startTime > 2) {
      feeder.setPercent(1);
      shootTime = Timer.getFPGATimestamp();
      shot = true;
    }
    this.distToSpeakerMeters = Constants.getDistance(x, Constants.Physical.SPEAKER_Y, drive.getMT2OdometryX(), drive.getMT2OdometryY());
    Logger.recordOutput("DistToSpeakerMeters", distToSpeakerMeters);
    Logger.recordOutput("Left RPM", Math.abs(shooter.getLeftShooterRPM() - left)/left < 0.1);
    Logger.recordOutput("Right RPM", Math.abs(shooter.getRightShooterRPM() + right)/right < 0.1);
    Logger.recordOutput("Theta Aligned", shooter.alignedPreset);
    Logger.recordOutput("Angle", Math.abs(shooter.getShooterAngle()-angle) < 2);
    Logger.recordOutput("Timeout", Timer.getFPGATimestamp() - startTime > 3);
    Logger.recordOutput("Has Shot", shot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.setShooterAngle(Constants.SetPoints.SHOOTER_DOWN_ANGLE_DEG);
    this.shooter.setShooterRPM(0.0, 0.0);
    this.feeder.setPercent(0.0);
    this.intake.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(Timer.getFPGATimestamp() - shootTime > 0.2 && shot) {
    //   return true;
    // } else return false;
    return false;
  }
}
