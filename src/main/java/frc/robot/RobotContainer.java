package frc.robot;

import java.util.HashMap;
import java.util.List;

// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.Swerve.*;
import frc.robot.commands.Combo.*;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.LEDs.Lights;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Pneumatics.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Subsystems */
    private final Swerve m_swerve = new Swerve();
    // private final Arm m_arm = new Arm();
    private final ArmProfiled m_arm = new ArmProfiled();
    // private final Elevator m_liftold = new Elevator();
    private final ElevatorProfiled m_lift = new ElevatorProfiled();
    private final Pneumatics m_air = new Pneumatics();
    // private final Limelight m_light = new Limelight();
    private final LEDs m_led = new LEDs();

    /* Controllers */
    private final Joystick m_driver = new Joystick(Constants.kDriverPort);
    private final Joystick m_operator = new Joystick(Constants.kOperatorPort);

    // private final AutoBuilder m_autoBuilder;
    private final HashMap<String, Command> m_eventMap = new HashMap<>();

    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
    private final SendableChooser<Command> m_autoChooser2 = new SendableChooser<>();
    // private final SendableChooser<List<PathPlannerTrajectory>> m_autoChooser = new SendableChooser<>();


    // // private final List<PathPlannerTrajectory> m_testPath = PathPlanner.loadPathGroup("Test Path", new PathConstraints(4, 3));
    // private final List<PathPlannerTrajectory> m_transPath = PathPlannerPath.loadPathGroup("Translation Path", new PathConstraints(4, 3));
    // private final List<PathPlannerTrajectory> m_rotPath = PathPlannerPath.loadPathGroup("Rotation Path", new PathConstraints(1, 1));
    // // private final List<PathPlannerTrajectory> m_dancePaths = PathPlanner.loadPathGroup("Dance Path", new PathConstraints(3, 3));
    // private final List<PathPlannerTrajectory> m_oneLong = PathPlannerPath.loadPathGroup("One Peice Long Path", new PathConstraints(1, 1), new PathConstraints(3, 2));
    // private final List<PathPlannerTrajectory> m_oneShort = PathPlannerPath.loadPathGroup("One Peice Short Path", new PathConstraints(2, 1));
    // // private final List<PathPlannerTrajectory> m_twoShort = PathPlanner.loadPathGroup("Two Peice Short Path", new PathConstraints(2, 1), new PathConstraints(2, 2), new PathConstraints(1.75, 2));
    // private final List<PathPlannerTrajectory> m_oneBal = PathPlannerPath.loadPathGroup("One Cone Balance", new PathConstraints(2, 2), new PathConstraints(1.25, 2), new PathConstraints(0.65, 2));
    // private final List<PathPlannerTrajectory> m_cubeTwoShort = PathPlannerPath.loadPathGroup("New Two Short", new PathConstraints(2, 2), new PathConstraints(2, 2), new PathConstraints(2, 2), new PathConstraints(2, 2), new PathConstraints(1, 2),new PathConstraints(2, 2));
    // private final List<PathPlannerTrajectory> m_fullBalance = PathPlannerPath.loadPathGroup("New Balance", new PathConstraints(1, 1));
    // private final List<PathPlannerTrajectory> m_moBalance = PathPlannerPath.loadPathGroup("New Mo Balance", new PathConstraints(2, 1));

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_swerve.setDefaultCommand(
            new TeleopSwerve(
                () -> -m_driver.getRawAxis(1),     // Translation
                () -> -m_driver.getRawAxis(0),     // Strafe
                () -> -m_driver.getRawAxis(2),    // Rotation
                () -> m_driver.getRawButton(5), // Field Centric
                m_swerve
            )
        );

        // m_arm.setDefaultCommand(new AnalongArmMove(() -> m_driver.getLeftTriggerAxis(), () -> m_driver.getRightTriggerAxis(), m_arm));

        m_eventMap.put("test1", new PrintCommand("Test 1"));
        m_eventMap.put("test2", new PrintCommand("Test 2"));
        m_eventMap.put("test3", new PrintCommand("Test 3"));
        m_eventMap.put("timeout", new WaitCommand(1));
        m_eventMap.put("secHalf", new WaitCommand(2.25));
        m_eventMap.put("openClaw", new OpenClaw(m_air));
        m_eventMap.put("closeClaw", new CloseClaw(m_air));
        m_eventMap.put("clawDown", new ClawDown(m_air));
        m_eventMap.put("clawUp", new ClawUp(m_air));
        m_eventMap.put("stowArm", new StowArm(m_arm, m_lift, m_air, m_swerve));
        m_eventMap.put("stage1", new Stage1(m_arm, m_lift, m_air, m_swerve));
        m_eventMap.put("stage2", new Stage2(m_arm, m_lift, m_air, m_swerve));
        m_eventMap.put("stage3", new Stage3(m_arm, m_lift, m_air, m_swerve));
        m_eventMap.put("cube3", new Cube3(m_arm, m_lift, m_air, m_swerve));
        m_eventMap.put("groundPick", new FloorHorizontal(m_arm, m_lift, m_air, m_swerve));
        m_eventMap.put("lowerCone", new LowerCone(m_lift));
        m_eventMap.put("flipGyro", new InstantCommand(() -> m_swerve.zeroGyro(180)));

        m_led.setDefaultCommand(new Lights(m_led));

        // m_autoBuilder = new SwerveAutoBuilder(
        //     m_swerve::getPose,
        //     m_swerve::resetOdometry,
        //     SwerveConstants.swerveKinematics,
        //     new PIDConstants(17, 0, 0),
        //     // new PIDConstants(8.5, 0, 0),
        //     new PIDConstants(6, 0, 0),
        //     m_swerve::setModuleStates,
        //     m_eventMap,
        //     true,
        //     m_swerve
        // );
        
        m_tab.add("Auton List", m_autoChooser2).withPosition(3, 2).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
        // m_autoChooser.setDefaultOption("Test Path", m_testPath);
        // m_autoChooser2.addOption("Translation Path", m_autoBuilder.fullAuto(m_transPath));
        // m_autoChooser2.addOption("Rotation Path", m_autoBuilder.fullAuto(m_rotPath));
        // // m_autoChooser.addOption("Dance Path", m_dancePaths);
        // m_autoChooser2.addOption("One Peice Long", m_autoBuilder.fullAuto(m_oneLong));
        // m_autoChooser2.addOption("One Peice Short", m_autoBuilder.fullAuto(m_oneShort));
        // // m_autoChooser2.addOption("Two Short", m_autoBuilder.fullAuto(m_twoShort));
        // // m_autoChooser.addOption("Two Short", m_twoShort);
        // // m_autoChooser.addOption("Two Long", m_twoLong);
        // // m_autoChooser.setDefaultOption("One & Balance", m_oneBal);
        // m_autoChooser2.setDefaultOption("One & Balance", m_autoBuilder.fullAuto(m_oneBal).andThen(new Balance(m_swerve)));
        // m_autoChooser2.addOption("New Balance", m_autoBuilder.fullAuto(m_fullBalance).andThen(new NewBalance(m_swerve)));
        // m_autoChooser2.addOption("New Mo Balance", m_autoBuilder.fullAuto(m_moBalance).andThen(new NewBalance(m_swerve)));
        // m_autoChooser2.addOption("Cube 2 Short", m_autoBuilder.fullAuto(m_cubeTwoShort));
        m_autoChooser2.setDefaultOption("Do Nothing", new WaitCommand(15));

        // PathPlannerServer.startServer(5811);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        new POVButton(m_driver, 180).whileTrue(new Down(m_lift));
        new POVButton(m_driver, 0).whileTrue(new Up(m_lift));
        new POVButton(m_driver, 90).whileTrue(new DownArm(m_arm));
        new POVButton(m_driver, 270).whileTrue(new UpArm(m_arm));

        // new JoystickButton(m_driver, XboxController.Button.kA.value).onTrue(new BuddyDown(m_air)).onFalse(new BuddyUp(m_air));

        new JoystickButton(m_driver, 6).toggleOnFalse(new SlowDrive(m_swerve));
        // new JoystickButton(m_driver, XboxController.Button.kB.value).whileTrue(new OrbitPiece(m_swerve, m_arm));
        new JoystickButton(m_driver, 1).onTrue(new ToggleTilt(m_air));
        new JoystickButton(m_driver, 14).onTrue(new ZeroGyro(m_swerve));
        new JoystickButton(m_driver, 9).onTrue(new ResetEncoders(m_swerve));
        new JoystickButton(m_driver, 2).whileTrue(new OpenClaw(m_air)).whileFalse(new CloseClaw(m_air));
        // new JoystickButton(m_driver, XboxController.Button.kB.value).onTrue(new ToggleClaw(m_air));

        // new JoystickButton(m_operator, 5).whileTrue(new BalanceProfiled(m_swerve));
        new JoystickButton(m_operator, 5).whileTrue(new SequentialCommandGroup(new FastBalance(m_swerve), new Balance(m_swerve)));

        new JoystickButton(m_operator, 2).onTrue(new StowArm(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 1).onTrue(new SlideStage(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 3).onTrue(new DropStage(m_arm, m_lift, m_air, m_swerve));
        // new JoystickButton(m_operator, 5).onTrue(new ClawUp(m_air));
        new JoystickButton(m_operator, 11).onTrue(new Stage1(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 13).onTrue(new Stage2(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 15).onTrue(new Stage3(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 12).onTrue(new FloorHorizontal(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 14).onTrue(new FloorVertical(m_arm, m_lift, m_air, m_swerve));
        new JoystickButton(m_operator, 4).onTrue(new RaiseCone(m_lift));
        new JoystickButton(m_operator, 9).onTrue(new LowerCone(m_lift));
        // new JoystickButton(m_operator, 10).onTrue(new ClawDown(m_air));
        new JoystickButton(m_operator, 10).onTrue(new BuddyDown(m_air)).onFalse(new BuddyUp(m_air));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // System.out.println(DriverStation.getAlliance());
        // return m_autoBuilder.fullAuto(m_autoChooser.getSelected()).andThen(new Balance(m_swerve));
        return m_autoChooser2.getSelected();
    }
}
