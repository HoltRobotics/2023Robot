// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Arm.SetAngle;
import frc.robot.commands.Elevator.SetHeight;
import frc.robot.commands.Pneumatics.ClawDown;
import frc.robot.commands.Pneumatics.CloseClaw;
import frc.robot.subsystems.ArmProfiled;
import frc.robot.subsystems.ElevatorProfiled;
import frc.robot.subsystems.Pneumatics;

public class StowArm extends ParallelCommandGroup {
  /**
   * Combo command that ses the arm, elevator, and claw to the default position. Protects the game peice when we are driving.
   * @param arm The Arm Subsystem
   * @param lift The Elevator Subsystem
   * @param air The Pneumatics Subsystem
   */
  public StowArm(ArmProfiled arm, ElevatorProfiled lift, Pneumatics air) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetHeight(ElevatorConstants.kStowHeight, lift), // Sets the elevator to the default height.
      new SetAngle(ArmConstants.kStowAngle, arm), // Sets the arm to the default angle.
      new CloseClaw(air), // Makes sure the claw is closed.
      new ClawDown(air) // Sets the claw pointing down.
    );
  }
}
