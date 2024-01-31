// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class Balance extends Command {
  private final PIDController m_controller;
  private final Swerve m_swerve;
  private final SwerveModuleState[] m_states;
  // private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  // private boolean m_isPID = false;

  /** Creates a new Balance. */
  public Balance(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = new PIDController(0, 0, 0);
    m_controller.setSetpoint(0);
    m_controller.setTolerance(5);
    // m_tab.add("Balance", m_controller);
    m_swerve = swerve;
    addRequirements(m_swerve);
    SwerveModuleState[] states = { // The states of each of the wheels that we want to set.
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)), // Front Left
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), // Front Right
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)), // Back Left
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)) // Back Right
    };
    m_states = states;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.print("Pitch: " + m_swerve.getPitch() + " ");
    // if(!m_isPID) {
    //   if(m_swerve.getPitch() > 14) {
    //     m_isPID = !m_isPID;
    //   }
    //   m_swerve.drive(new Translation2d(1.5, 0), 0, true, true);
    //   System.out.println("NO PID");
    // } else {
    //   m_swerve.drive(new Translation2d(-m_controller.calculate(m_swerve.getPitch()) * 0.5, 0), 0, true, true);
    //   System.out.println("YES PID");
    // }
    // m_swerve.drive(new Translation2d(-m_controller.calculate(m_swerve.getPitch()) * 0.5, 0), 0, true, true);
    System.out.println("Slow: " + m_swerve.getPitch());
    if(Math.abs(m_swerve.getPitch()) < 8) {
      m_swerve.setModuleStates(m_states);
      // m_swerve.drive(new Translation2d(), Math.toRadians(90), true, true);
    } else {
      m_swerve.drive(new Translation2d(-0.55, 0), 0, true, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("STOPPED");
    m_swerve.stopDrive();
    // m_isPID = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_controller.atSetpoint();
    return false;
  }
}
