// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Arm.SetAngle;
import frc.robot.commands.Elevator.SetHeight;
import frc.robot.commands.Pneumatics.ClawUp;
import frc.robot.commands.Pneumatics.CloseClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pneumatics;

public class Stage2 extends ParallelCommandGroup {
   /**
   * Combo command that sets the arm, elevator, and claw to the right states to score on the mid goal. Runs all the commands at once.
   * @param arm The Arm Subsystem
   * @param lift The Elevator Subsystem
   * @param air The Pneumatics Subsystem
   */
  public Stage2(Arm arm, Elevator lift, Pneumatics air) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup( // Runs theses commands one after another.
        new SetHeight(ElevatorConstants.kStage2Height + 0.1, lift), // Sets the command to a little higher then needed, So the cone won't hit the post.
        new WaitCommand(0.5), // Waits 0.5 seconds for the elevator to get to the first height.
        new SetHeight(ElevatorConstants.kStage2Height, lift) // Lowers the elevator to the final height.
        ),
      new SetAngle(ArmConstants.kStage2Angle, arm), // Sets the arm to the right angle.
      new CloseClaw(air), // Makes sure the claw is closed.
      new ClawUp(air) // Sets the claw level.
    );
  }
}
