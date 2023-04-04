// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight"); // Makes the Limelight data table.
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main"); // Gets the Shuffleboard tab.

  private double[] m_botPose = m_table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]); // Makes a double array to hold the robot pose.
  private final NetworkTableEntry tv = m_table.getEntry("tv"); // Makes a object that tells if the limelight has a target or not.

  private final HttpCamera m_camera; // Makes a camera object.

  private final GenericEntry m_tv; // Makes a shuffleboard object that tells if the limelight has a target.

  /** Creates a new Limelight. */
  public Limelight() {
    m_tv = m_tab.add("Has Target", false).withPosition(5, 1).withSize(1, 1).getEntry(); // Adds a box to the shuffleboard to say if the limelight has a target.
    m_camera = new HttpCamera("LimeLight", "http://10.60.78.11:5800/", HttpCameraKind.kMJPGStreamer); // Sets the camera object to the limelight camera.
    CameraServer.startAutomaticCapture(m_camera); // Starts the camera.
    m_tab.add("Camera", m_camera).withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 7).withSize(3, 3); // Adds the camera to the shuffleboard.
  }

  /**
   * Gets the current robot pose 3D from the limelight.
   * @return a Pose3d of the robot's current position.
   */
  public Pose3d getPose3D() {
    return new Pose3d( // Makes the Pose3D
      new Translation3d(m_botPose[0], m_botPose[1], m_botPose[2]), // Adds the translation to the pose.
      new Rotation3d(Units.degreesToRadians(m_botPose[3]), Units.degreesToRadians(m_botPose[4]), Units.degreesToRadians(m_botPose[5])) // Adds the rotation to the pose.
    );
  }

  /**
   * Gets the current distance from the target.
   * @return the distance in meters.
   */
  public double getDistance() {
    return m_botPose[1]; // Returns the current distance.
  }

  /**
   * This method runs every 20ms while the robot is on.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_botPose = m_table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]); // Updates the robot pose.
    m_tv.setBoolean(tv.getDouble(0) == 1); // Updates shuffleboard on if the limelight has a target.
  }
}
