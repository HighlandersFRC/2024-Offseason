package frc.robot.subsystems;

import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.json.JSONArray;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.tools.math.Vector;

public class Peripherals {
  private PhotonCamera frontCam = new PhotonCamera("9281_Front");

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private Pigeon2 pigeon = new Pigeon2(0, "Canivore");
  private Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
  Transform3d robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.5), new Rotation3d(0, 0, 0));

  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
      PoseStrategy.AVERAGE_BEST_TARGETS, robotToCam);

  public Peripherals() {
  }

  /**
   * Initializes the Peripherals subsystem.
   * 
   * This method sets up the IMU configuration, mount pose, and zeroes the IMU.
   * It also applies the default command to the Peripherals subsystem.
   */
  public void init() {
    // Set the mount pose configuration for the IMU
    pigeonConfig.MountPose.MountPosePitch = -85.28813934326172;
    pigeonConfig.MountPose.MountPoseRoll = 32.49883270263672;
    pigeonConfig.MountPose.MountPoseYaw = 0.1901332437992096;

    // Apply the IMU configuration
    pigeon.getConfigurator().apply(pigeonConfig);

    // Zero the IMU angle
    zeroPigeon();
  }

  public double getFrontCamYaw() {
    double yaw = 0.0;
    var result = frontCam.getLatestResult();
    Logger.recordOutput("has target", result.hasTargets());
    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      PhotonTrackedTarget target = result.getBestTarget();
      yaw = target.getYaw();
    }
    return yaw;
  }

  public double getFrontCamPitch() {
    double pitch = 0.0;
    var result = frontCam.getLatestResult();
    if (result.hasTargets()) {

      List<PhotonTrackedTarget> targets = result.getTargets();
      PhotonTrackedTarget target = result.getBestTarget();
      pitch = target.getPitch();
    }

    return pitch;
  }

  private Pose2d getRobotPoseViaTrig(PhotonTrackedTarget trackedTarget, double[] cameraPositionOnRobot,
      double robotAngle) {
    double pitch = trackedTarget.getPitch();
    double yaw = trackedTarget.getYaw();
    int id = trackedTarget.getFiducialId();
    // double cameraYaw = 0.0;
    // double cameraYOffset = 0.0;
    // double cameraPitch = 0.0;
    // double cameraHeight = Constants.inchesToMeters(30.68);
    double cameraXOffset = cameraPositionOnRobot[0];
    double cameraYOffset = cameraPositionOnRobot[1];
    double cameraZOffset = cameraPositionOnRobot[2];
    double cameraRXOffset = cameraPositionOnRobot[3];
    double cameraRYOffset = cameraPositionOnRobot[4];
    double cameraRZOffset = cameraPositionOnRobot[5];

    double[] tagPose = Constants.Vision.TAG_POSES[id - 1];
    double tagHeight = tagPose[2];
    double tagX = tagPose[0];
    double tagY = tagPose[1];

    double distToTag = (tagHeight - cameraZOffset) / Math.tan(Math.toRadians(pitch + cameraRYOffset));
    Logger.recordOutput("Distance to Tag", distToTag);
    Logger.recordOutput("yaw to Tag", yaw);
    // double txProjOntoGroundPlane = Math.atan((Math.tan(yaw)) / Math.cos(pitch));
    double xFromTag = distToTag * Math.cos(Math.toRadians(yaw));
    double yFromTag = distToTag * Math.sin(Math.toRadians(yaw));
    Logger.recordOutput("x to Tag", xFromTag);
    Logger.recordOutput("y to Tag", yFromTag);

    double fieldPoseX = -xFromTag + tagX;
    double fieldPoseY = yFromTag + tagY - cameraYOffset;
    Pose2d pose = new Pose2d(fieldPoseX, fieldPoseY, new Rotation2d(getPigeonAngle()));
    return pose;
  }

  public Pose2d getFrontCamTrigPose() {
    var result = frontCam.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      Pose2d robotPose = getRobotPoseViaTrig(target, Constants.Vision.FRONT_CAMERA_POSE, getPigeonAngle());
      Logger.recordOutput("Trig Localiazation", robotPose);
      return robotPose;
    } else {
      Pose2d defaultPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
      return defaultPose;
    }
  }

  public Pose3d getFrontCamPnPPose() {
    var result = frontCam.getLatestResult();
    Optional<EstimatedRobotPose> multiTagResult = photonPoseEstimator.update(result);
    if (multiTagResult.isPresent()) {
      Pose3d robotPose = multiTagResult.get().estimatedPose;
      Logger.recordOutput("multitag result", robotPose);
      // double[] pose = { robotPose.getX(), robotPose.getY(),
      // robotPose.getRotation().getAngle() };

      return robotPose;
    } else {
      Pose3d robotPose = new Pose3d();
      return robotPose;
    }
  }

  public double getFrontCamLatency() {
    return frontCam.getLatestResult().getTimestampSeconds();
  }

  public double getFrontCamAmbiguity() {
    return frontCam.getLatestResult().getBestTarget().poseAmbiguity;
  }

  /**
   * Sets the IMU angle to 0
   */
  public void zeroPigeon() {
    setPigeonAngle(0.0);
  }

  /**
   * Sets the angle of the IMU
   * 
   * @param degrees - Angle to be set to the IMU
   */
  public void setPigeonAngle(double degrees) {
    pigeon.setYaw(degrees);
  }

  /**
   * Retrieves the yaw of the robot
   * 
   * @return Yaw in degrees
   */
  public double getPigeonAngle() {
    return pigeon.getYaw().getValueAsDouble();
  }

  /**
   * Retrieves the absolute angular velocity of the IMU's Z-axis in device
   * coordinates.
   *
   * @return The absolute angular velocity of the IMU's Z-axis in device
   *         coordinates.
   *         The value is in radians per second.
   */
  public double getPigeonAngularVelocity() {
    return Math.abs(pigeon.getAngularVelocityZDevice().getValueAsDouble());
  }

  /**
   * Retrieves the absolute angular velocity of the IMU's Z-axis in world
   * coordinates.
   *
   * @return The absolute angular velocity of the IMU's Z-axis in world
   *         coordinates.
   *         The value is in radians per second.
   */
  public double getPigeonAngularVelocityW() {
    return pigeon.getAngularVelocityZWorld().getValueAsDouble();
  }

  /**
   * Retrieves the acceleration vector of the robot
   * 
   * @return Current acceleration vector of the robot
   */
  public Vector getPigeonLinAccel() {
    Vector accelVector = new Vector();
    accelVector.setI(pigeon.getAccelerationX().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
    accelVector.setJ(pigeon.getAccelerationY().getValueAsDouble() / Constants.Physical.GRAVITY_ACCEL_MS2);
    return accelVector;
  }

  public void periodic() {
    getFrontCamPnPPose();
    var result = frontCam.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      Pose2d robotPose = getRobotPoseViaTrig(target, Constants.Vision.FRONT_CAMERA_POSE, getPigeonAngle());
      Logger.recordOutput("Trig Localiazation", robotPose);
    }
  }
}