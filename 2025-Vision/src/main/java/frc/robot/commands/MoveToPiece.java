// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Peripherals;
import frc.robot.tools.controlloops.PID;
import frc.robot.tools.math.Vector;

public class MoveToPiece extends Command {
  private Drive drive;
  private Peripherals peripherals;

  private PID pid;
  private double kP = 3;
  private double kI = 0;
  private double kD = 0;
  private double[] desiredVelocityArray = new double[3];
  private double desiredThetaChange = 0;

  private double ty, tx;
  private double noteX, noteY;

  /** Creates a new MoveToPiece. */
  public MoveToPiece(Drive drive, Peripherals peripherals) {
    this.drive = drive;
    this.peripherals = peripherals;
    // addRequirements(drive, peripherals);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PID(kP, kI, kD);
    pid.setSetPoint(0);
    pid.setMinOutput(-4);
    pid.setMaxOutput(4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = peripherals.getBackCamYaw();
    ty = peripherals.getBackCamPitch();

    double cameraHeight = Constants.inchesToMeters(29.5);
    double cameraAngle = 16.1;
    double cameraXOffset = -Constants.inchesToMeters(0.0);

    double robotX = drive.getFusedOdometryX();
    double robotY = drive.getFusedOdometryY();
    double robotAngle = Constants.standardizeAngleDegrees(peripherals.getPigeonAngle());

    double targetDistance = (cameraHeight) / Math.tan(Math.toRadians(ty - cameraAngle));

    noteY = robotY + (-targetDistance * Math.sin(Math.toRadians(tx) + robotAngle));
    noteX = robotX + ((targetDistance * Math.cos(Math.toRadians(tx) + robotAngle)) - cameraXOffset);

    Logger.recordOutput("Note X: ", noteX);
    Logger.recordOutput("Note Y: ", noteY);

    // desiredVelocityArray = drive.driveToPoint(drive.getFusedOdometryX(),
    // drive.getFusedOdometryY(), 0.0, 0.1, noteX,
    // noteY);

    // Vector velocityVector = new Vector();
    // velocityVector.setI(desiredVelocityArray[0]);
    // velocityVector.setJ(desiredVelocityArray[1]);
    // desiredThetaChange = desiredVelocityArray[2];
    // velocityVector.setI(0);
    // velocityVector.setJ(0);
    // desiredThetaChange = 0;

    // drive.autoDrive(velocityVector, desiredThetaChange);

    // double angleToPiece = peripherals.getBackCamTargetTx();
    // pid.updatePID(angleToPiece);
    // double result = -pid.getResult();

    // drive.autoRobotCentricDrive(new Vector(-3, 0), result);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Vector velocityVector = new Vector();
    // velocityVector.setI(0);
    // velocityVector.setJ(0);
    // double desiredThetaChange = 0.0;
    // drive.autoDrive(velocityVector, desiredThetaChange);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (!intake.getBeamBreak()) {
    // return true;
    // } else {
    return false;
    // }
  }
}