package frc.robot;

import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DoNothing;
import frc.robot.commands.FullSendFollower;
import frc.robot.commands.MoveTestMotor;
import frc.robot.commands.PolarAutoFollower;
import frc.robot.commands.SetElevatorPercent;
import frc.robot.commands.SetRobotState;
import frc.robot.commands.ZeroAngleMidMatch;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MotorTest;
import frc.robot.subsystems.Peripherals;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  Peripherals peripherals = new Peripherals();
  Drive drive = new Drive(peripherals);
  Elevator elevator = new Elevator();
  MotorTest motorTest = new MotorTest();
  Lights lights = new Lights();
  Superstructure superstructure = new Superstructure(drive, elevator);

  HashMap<String, Supplier<Command>> commandMap = new HashMap<String, Supplier<Command>>() {
    {
      put("Instant", () -> new InstantCommand());
      put("Wait", () -> new DoNothing());
      put("Print", () -> new PrintCommand("10s"));
      put("Full Send", () -> new FullSendFollower(drive, null, false));
      put("Place Coral High", () -> new ParallelCommandGroup(new SetRobotState(superstructure, SuperState.ELEVATOR_UP), new SequentialCommandGroup(new WaitCommand(1.0), new MoveTestMotor(motorTest, "motor1", 0.3))));
      put("Intake Coral", () -> new MoveTestMotor(motorTest, "motor1", -0.3));
    }
  };

  File[] autoFiles = new File[Constants.Autonomous.paths.length];
  Command[] autos = new Command[Constants.Autonomous.paths.length];
  JSONObject[] autoJSONs = new JSONObject[Constants.Autonomous.paths.length];
  JSONArray[] autoPoints = new JSONArray[Constants.Autonomous.paths.length];

  HashMap<String, BooleanSupplier> conditionMap = new HashMap<String, BooleanSupplier>() {
    {
    }
  };

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // Load the Path Files
    for (int i = 0; i < Constants.Autonomous.paths.length; i++) {
      try {
        autoFiles[i] = new File(Filesystem.getDeployDirectory().getPath() + "/" + Constants.Autonomous.paths[i]);
        FileReader scanner = new FileReader(autoFiles[i]);
        autoJSONs[i] = new JSONObject(new JSONTokener(scanner));
        autoPoints[i] = (JSONArray) autoJSONs[i].getJSONArray("paths").getJSONObject(0).getJSONArray("sampled_points");
        autos[i] = new PolarAutoFollower(autoJSONs[i], drive, lights, peripherals, commandMap, conditionMap);
        System.out.println("Loaded Path: " + Constants.Autonomous.paths[i]);
      } catch (Exception e) {
        System.out.println("ERROR LOADING PATH " + Constants.Autonomous.paths[i] + ":" + e);
      }
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // COMPETITION CONTROLS
    // Driver

    OI.driverViewButton.whileTrue(new ZeroAngleMidMatch(drive));
    OI.driverA.whileTrue(new SetElevatorPercent(elevator, 0.3));
    OI.driverY.whileTrue(new SetElevatorPercent(elevator, -0.3));
    OI.driverLT.whileTrue(new MoveTestMotor(motorTest, "motor1", 0.3));
    OI.driverRT.whileTrue(new MoveTestMotor(motorTest, "motor1", -0.3));
    OI.driverX.whileTrue(new SetRobotState(superstructure, SuperState.ELEVATOR_MID));
    OI.driverB.whileTrue(new SetRobotState(superstructure, SuperState.ELEVATOR_UP));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    int selectedPath = Constants.Autonomous.getSelectedPathIndex();
    if (selectedPath >= Constants.Autonomous.paths.length) {
      selectedPath = -1;
    }
    if (selectedPath == -1) {
      System.out.println("Selected Path: None");
      return new DoNothing();
    } else {
      this.drive.autoInit(autoPoints[selectedPath]);
      System.out.println("Selected Path: " + Constants.Autonomous.paths[selectedPath]);
      return this.autos[selectedPath];
    }
  }
}
