package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

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
    private final Arm m_arm = new Arm();
    private final Elevator m_lift = new Elevator();
    private final Pneumatics m_air = new Pneumatics();
    private final Limelight m_light = new Limelight();

    /* Controllers */
    private final XboxController m_driver = new XboxController(Constants.kDriverPort);
    private final Joystick m_operator = new Joystick(Constants.kOperatorPort);

    private final SwerveAutoBuilder m_autoBuilder;
    private final HashMap<String, Command> m_eventMap = new HashMap<>();

    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
    private final SendableChooser<List<PathPlannerTrajectory>> m_autoChooser = new SendableChooser<>();

    // private final UsbCamera m_camera;

    private final List<PathPlannerTrajectory> m_testPath = PathPlanner.loadPathGroup("Test Path", new PathConstraints(4, 3));
    private final List<PathPlannerTrajectory> m_transPath = PathPlanner.loadPathGroup("Translation Path", new PathConstraints(3, 3));
    private final List<PathPlannerTrajectory> m_rotPath = PathPlanner.loadPathGroup("Rotation Path", new PathConstraints(1, 1));
    private final List<PathPlannerTrajectory> m_dancePaths = PathPlanner.loadPathGroup("Dance Path", new PathConstraints(3, 3));
    private final List<PathPlannerTrajectory> m_LPath = PathPlanner.loadPathGroup("L Path", new PathConstraints(3, 3));
    private final List<PathPlannerTrajectory> m_oneLong = PathPlanner.loadPathGroup("One Peice Long Path", new PathConstraints(1, 1), new PathConstraints(3, 2));
    private final List<PathPlannerTrajectory> m_oneShort = PathPlanner.loadPathGroup("One Peice Short Path", new PathConstraints(1, 1), new PathConstraints(3, 2));
    private final List<PathPlannerTrajectory> m_twoShort = PathPlanner.loadPathGroup("Two Peice Short Path", new PathConstraints(1, 1), new PathConstraints(3, 2), new PathConstraints(3, 2));

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_swerve.setDefaultCommand(
            new TeleopSwerve(
                () -> -m_driver.getLeftY(),     // Translation
                () -> -m_driver.getLeftX(),     // Strafe
                () -> -m_driver.getRightX(),    // Rotation
                () -> m_driver.getLeftBumper(), // Field Centric
                m_swerve
            )
        );

        m_tab.add("Auton List", m_autoChooser).withPosition(3, 2).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
        m_autoChooser.setDefaultOption("Test Path", m_testPath);
        m_autoChooser.addOption("Translation Path", m_transPath);
        m_autoChooser.addOption("Rotation Path", m_rotPath);
        m_autoChooser.addOption("Dance Path", m_dancePaths);
        m_autoChooser.addOption("L", m_LPath);
        m_autoChooser.addOption("One Peice Long", m_oneLong);
        m_autoChooser.addOption("One Peice Short", m_oneShort);
        m_autoChooser.addOption("Two Short", m_twoShort);

        PathPlannerServer.startServer(5811);

        // m_camera = CameraServer.startAutomaticCapture();

        // m_tab.add("Camera", m_camera).withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 7).withSize(3, 3);

        m_eventMap.put("test1", new PrintCommand("Test 1"));
        m_eventMap.put("test2", new PrintCommand("Test 2"));
        m_eventMap.put("test3", new PrintCommand("Test 3"));
        m_eventMap.put("timeout", new WaitCommand(1));
        m_eventMap.put("openClaw", new OpenClaw(m_air));
        m_eventMap.put("closeClaw", new CloseClaw(m_air));
        m_eventMap.put("clawDown", new ClawDown(m_air));
        m_eventMap.put("clawUp", new ClawUp(m_air));
        m_eventMap.put("stowArm", new StowArm(m_arm, m_lift, m_air));
        m_eventMap.put("stage1", new Stage1(m_arm, m_lift, m_air));
        m_eventMap.put("stage2", new Stage2(m_arm, m_lift, m_air));
        m_eventMap.put("stage3", new Stage3(m_arm, m_lift, m_air));

        m_autoBuilder = new SwerveAutoBuilder(
            m_swerve::getPose,
            m_swerve::resetOdometry,
            SwerveConstants.swerveKinematics,
            new PIDConstants(9, 0, 0), //TODO: tune theses
            new PIDConstants(12, 0, 0),
            m_swerve::setModuleStates,
            m_eventMap,
            true,
            m_swerve
        );

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
        new POVButton(m_driver, 90).whileTrue(new UpArm(m_arm));
        new POVButton(m_driver, 270).whileTrue(new DownArm(m_arm));

        // new JoystickButton(m_driver, XboxController.Button.kA.value).onTrue(new BuddyDown(m_air)).onFalse(new BuddyUp(m_air));

        new JoystickButton(m_driver, XboxController.Button.kRightBumper.value).whileTrue(new SlowDrive(m_swerve));
        new JoystickButton(m_driver, XboxController.Button.kB.value).whileTrue(new OrbitPiece(m_swerve, m_arm));
        new JoystickButton(m_driver, XboxController.Button.kX.value).onTrue(new ToggleTilt(m_air));
        new JoystickButton(m_driver, XboxController.Button.kStart.value).onTrue(new ZeroGyro(m_swerve));
        new JoystickButton(m_driver, XboxController.Button.kBack.value).onTrue(new ResetEncoders(m_swerve));
        new JoystickButton(m_driver, XboxController.Button.kA.value).whileTrue(new OpenClaw(m_air)).onFalse(new CloseClaw(m_air));


        new JoystickButton(m_operator, 1).onTrue(new StowArm(m_arm, m_lift, m_air));
        new JoystickButton(m_operator, 2).onTrue(new SlideStage(m_arm, m_lift, m_air));
        new JoystickButton(m_operator, 3).onTrue(new DropStage(m_arm, m_lift, m_air));
        new JoystickButton(m_operator, 5).onTrue(new ClawUp(m_air));
        new JoystickButton(m_operator, 6).onTrue(new Stage1(m_arm, m_lift, m_air));
        new JoystickButton(m_operator, 7).onTrue(new Stage2(m_arm, m_lift, m_air));
        new JoystickButton(m_operator, 8).onTrue(new Stage3(m_arm, m_lift, m_air));
        new JoystickButton(m_operator, 10).onTrue(new ClawDown(m_air));
        new JoystickButton(m_operator, 21).onTrue(new BuddyDown(m_air)).onFalse(new BuddyUp(m_air));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_autoBuilder.fullAuto(m_autoChooser.getSelected());
    }
}
