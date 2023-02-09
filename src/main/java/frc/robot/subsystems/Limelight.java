// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");

  private double[] m_botPose = m_table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
  private final NetworkTableEntry tv = m_table.getEntry("tv");

  private final GenericEntry m_tv;

  //TODO: add limelight camera to the shuffleboard and get rid of the other camera
  /** Creates a new Limelight. */
  public Limelight() {
    m_tv = m_tab.add("Has Target", false).getEntry(); //TODO: add the proper location on shuffleboard
  }

  public Pose3d getPose3D() {
    return new Pose3d(
      new Translation3d(m_botPose[0], m_botPose[1], m_botPose[2]),
      new Rotation3d(Units.degreesToRadians(m_botPose[3]), Units.degreesToRadians(m_botPose[4]), Units.degreesToRadians(m_botPose[5]))
    );
  }

  public double getDistance() {
    return m_botPose[1];
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_botPose = m_table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    m_tv.setBoolean(tv.getDouble(0) == 1);
  }
}
