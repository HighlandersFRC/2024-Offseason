package frc.robot;

import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AlignedPresetShoot;
import frc.robot.commands.Amp;
import frc.robot.commands.AngleShooter;
import frc.robot.commands.AutoNonThetaShoot;
import frc.robot.commands.AutoPositionalShoot;
import frc.robot.commands.AutoSpinUp;
import frc.robot.commands.AutomaticallyIntake;
import frc.robot.commands.DoNothing;
import frc.robot.commands.DriveAutoAligned;
import frc.robot.commands.DriveThetaAligned;
import frc.robot.commands.MoveToPiece;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.PresetShoot;
import frc.robot.commands.ReverseFeeder;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.sensors.Proximity;
// import frc.robot.sensors.TOF;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Shooter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  //Sensors
  // private TOF tof = new TOF();
  private Proximity proximity = new Proximity();

  //Subsystems
  private Lights lights = new Lights();
  private Peripherals peripherals = new Peripherals();
  private Drive drive = new Drive(peripherals);
  private Intake intake = new Intake();
  private Feeder feeder = new Feeder();
  private Shooter shooter = new Shooter();
  HashMap<String, Supplier<Command>> commandMap = new HashMap<String, Supplier<Command>>() {
    {
      put("Instant", () -> new InstantCommand());
      put("Wait", () -> new DoNothing());
      put("Intake", () -> new RunIntake(intake, feeder, 0.6));
      put("Outtake", () -> new ReverseFeeder(intake, feeder, shooter));
      put("Auto Shoot", () -> new AutoPositionalShoot(intake, drive, shooter, feeder, peripherals, lights, 1200, 26, 7000, false));
      put("Track Target", () -> new AutoNonThetaShoot(intake, drive, shooter, feeder, peripherals, lights, 1200, 26, 7000, false));
      put("Preset Shot 1", () -> new PresetShoot(intake, shooter, feeder, 6000, 3000, -45));
      put("Preset Shot 2", () -> new PresetShoot(intake, shooter, feeder, 6000, 3000, -55));
      put("Spin Up", () -> new AutoSpinUp(drive, shooter, peripherals, lights, 1200, 26, 7000, false));


    }
  };
  HashMap<String, BooleanSupplier> conditionMap = new HashMap<String, BooleanSupplier>() {
    {
      put("Note In Intake", () -> intake.getBeamBreak());
    }
  };
  // private Logger logger = Logger.getInstance();

  private double shooterAngleDegreesTuning = 0;
  private double shooterRPMTuning = 0;
  private double startTime = Timer.getFPGATimestamp();
  private boolean checkedCAN = false;

  File[] autoFiles = new File[Constants.paths.length];
  Command[] autos = new Command[Constants.paths.length];
  JSONObject[] autoJSONs = new JSONObject[Constants.paths.length];
  JSONArray[] autoPoints = new JSONArray[Constants.paths.length];

  String fieldSide = "blue";

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Shooter Angle Degrees (tuning)", 0);
    SmartDashboard.putNumber("Shooter RPM (input)", 0);
    // System.out.println("Starting");
    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    // new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    // Logger.recordMetadata("Code", "Running");
    // Logger.disableDeterministicTimestamps(); // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    // System.out.println("Started Logger");
    this.fieldSide = "blue";


    lights.init(fieldSide);
    peripherals.init();
    drive.init(fieldSide);
    feeder.init();
    shooter.init();

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    
    PortForwarder.add(5800, "limelight-front.local", 5800);
    PortForwarder.add(5801, "limelight-front.local", 5801);
    
    PortForwarder.add(5800, "limelight-back.local", 5800);
    PortForwarder.add(5801, "limelight-back.local", 5801);

    PortForwarder.add(5800, "limelight-left.local", 5800);
    PortForwarder.add(5801, "limelight-left.local", 5801);

    PortForwarder.add(5800, "limelight-right.local", 5800);
    PortForwarder.add(5801, "limelight-right.local", 5801);

    PortForwarder.add(5800, "10.99.99.41", 5800);
    PortForwarder.add(5801, "10.99.99.41", 5801);

    PortForwarder.add(5800, "10.99.99.42", 5800);
    PortForwarder.add(5801, "10.99.99.42", 5801);

    PortForwarder.add(5800, "10.99.99.43", 5800);
    PortForwarder.add(5801, "10.99.99.43", 5801);

    PortForwarder.add(5800, "10.99.99.44", 5800);
    PortForwarder.add(5801, "10.99.99.44", 5801);
    lights.clearAnimations();
    lights.setCommandRunning(true);
    lights.setRGBFade();

    for (int i = 0; i < Constants.paths.length; i++){
      try {
        autoFiles[i] = new File(Filesystem.getDeployDirectory().getPath() + "/" + Constants.paths[i]);
        FileReader scanner = new FileReader(autoFiles[i]);
        autoJSONs[i] = new JSONObject(new JSONTokener(scanner));
        autoPoints[i] = (JSONArray) autoJSONs[i].getJSONArray("paths").getJSONObject(0).getJSONArray("sampled_points");
        autos[i] = new PolarAutoFollower(autoJSONs[i], drive, lights, peripherals, commandMap, conditionMap);
      } catch (Exception e) {
        System.out.println("ERROR LOADING PATH "+Constants.paths[i]+":" + e);
      }
    }
    OI.init();
    System.out.println("end robot init");
  }
 
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Raw Shooter Angle", shooter.getRawShooterAngle());
    SmartDashboard.putBoolean("Beam Break", intake.getBeamBreak());
    SmartDashboard.putNumber("Shooter Angle", shooter.getShooterAngle());
    shooterAngleDegreesTuning = SmartDashboard.getNumber("Shooter Angle Degrees (tuning)", 0);
    shooterRPMTuning = SmartDashboard.getNumber("Shooter RPM (input)", 0);
    CommandScheduler.getInstance().run();

    // try{
    //   Logger.recordOutput("Localization Odometry", drive.getLocalizationOdometry());
    // } catch(Exception e) {
    //   System.out.println("Problem with logging");
    // }

    // try{
    //   Logger.recordOutput("Wheel Odometry", drive.getOdometry());
    // } catch(Exception e) {
    //   System.out.println("Problem with logging");
    // }

    try{
      Logger.recordOutput("MT2 Odometry", drive.getMT2Odometry());
    } catch(Exception e) {
      System.out.println("Problem with logging");
    }
    Logger.recordOutput("Swerve Module States", drive.getModuleStates());
    Logger.recordOutput("Swerve Module Setpoints", drive.getModuleSetpoints());
    Logger.recordOutput("IMU", peripherals.getPigeonAngle());
    Logger.recordOutput("Left Shooter Speed", shooter.getLeftShooterRPM());
    Logger.recordOutput("Right Shooter Speed", shooter.getRightShooterRPM());
    
    Constants.periodic();
    lights.periodic();
    // tof.periodic();
    proximity.periodic();

    // drive.periodic(); // remove for competition
    peripherals.periodic();
    
    // System.out.println("0-1: " + (t1 - t0));

    // SmartDashboard.putNumber("Carriage Rotation", climber.getCarriageRotationDegrees());
  }

  @Override
  public void disabledInit() {
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    OI.operatorController.setRumble(RumbleType.kBothRumble, 0);
    lights.clearAnimations();
    lights.setCommandRunning(true);
    lights.setRainbow();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    if (OI.isBlueSide()) {
      System.out.println("ON BLUE SIDE");
      fieldSide = "blue";
    } else {
      System.out.println("ON RED SIDE");
      fieldSide = "red";
    }
    this.drive.setFieldSide(fieldSide);
    this.peripherals.setFieldSide(fieldSide);
    System.out.print("Selected Auto: ");
    final int selectedPath = Constants.getSelectedPathIndex();
    if (selectedPath == -1) {
      System.out.println("Do Nothing");
      new DoNothing().schedule();
    } else {
      this.autos[selectedPath].schedule();
      this.drive.autoInit(autoPoints[selectedPath]);
      System.out.println(Constants.paths[selectedPath]);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override 
  public void teleopInit() {
    lights.setCommandRunning(false);
    lights.clearAnimations();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (OI.isBlueSide()) {
      fieldSide = "blue";
    } else {
      fieldSide = "red";
    }

    if (this.fieldSide == "red"){
      this.drive.setPigeonAfterAuto();
    }
    System.out.println("field side" + fieldSide);

    this.peripherals.setFieldSide(fieldSide);
    this.drive.setFieldSide(fieldSide);

    //CONTROLS
    double[] lookupTable = {shooterRPMTuning, shooterRPMTuning/2, shooterAngleDegreesTuning};
    //Driver
    OI.driverViewButton.whileTrue(new ZeroAngleMidMatch(drive));
    // OI.driverX.whileTrue(new DriveAutoAligned(drive, peripherals));
    OI.driverRT.whileTrue(new RunIntake(intake, feeder, 0.6));
    // OI.driverA.whileTrue(new PresetShoot(shooter, feeder, lookupTable));
    // OI.driverA.whileTrue(new AlignedPresetShoot(shooter, feeder, drive, peripherals,
    // Constants.SetPoints.SHOOTER_PODIUM_PRESET));
    OI.driverA.whileTrue(new AutoPositionalShoot(intake, drive, shooter, feeder, peripherals, lights, 1200, 26, 7000, false));
    OI.driverB.onTrue(new Amp(shooter, drive, peripherals));
    OI.driverLT.whileTrue(new ReverseFeeder(intake, feeder, shooter));
    OI.driverY.whileTrue(new AutoNonThetaShoot(intake, drive, shooter, feeder, peripherals, lights, 1200, 26, 7000, false));
    OI.driverX.whileTrue(new PresetShoot(intake, shooter, feeder, 6000, 3000, -45));
  }

  @Override
  public void teleopPeriodic() {
    if(!intake.getBeamBreak()) {
      OI.driverController.setRumble(RumbleType.kBothRumble, 1);
    } else {
      OI.driverController.setRumble(RumbleType.kBothRumble, 0);
    }
    SmartDashboard.putNumber("Shooter Angle Target", shooter.getShooterAngleTarget());
    SmartDashboard.putNumber("Left RPM", shooter.getLeftShooterRPM());
    SmartDashboard.putNumber("Right RPM", shooter.getRightShooterRPM());
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
