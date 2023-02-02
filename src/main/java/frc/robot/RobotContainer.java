package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Swerve.*;
import frc.robot.commands.Pneumatics.*;
import frc.robot.subsystems.*;

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

    /* Controllers */
    private final XboxController m_driver = new XboxController(Constants.kDriverPort);
    private final Joystick m_operator = new Joystick(Constants.kOperatorPort);

    private final SwerveAutoBuilder m_autoBuilder;
    private final HashMap<String, Command> m_eventMap = new HashMap<>();

    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
    private final SendableChooser<PathPlannerTrajectory> m_autoChooser = new SendableChooser<>();

    private final UsbCamera m_camera;

    private final PathPlannerTrajectory m_testPath = PathPlanner.loadPath("Test Path", new PathConstraints(4, 3));
    private final PathPlannerTrajectory m_transPath = PathPlanner.loadPath("Translation Path", new PathConstraints(4, 3));
    private final PathPlannerTrajectory m_rotPath = PathPlanner.loadPath("Rotation Path", new PathConstraints(4, 3));

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

        m_camera = CameraServer.startAutomaticCapture();

        m_tab.add("Camera", m_camera).withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 7).withSize(3, 3);

        m_eventMap.put("test1", new PrintCommand("Test 1"));
        m_eventMap.put("test2", new PrintCommand("Test 2"));
        m_eventMap.put("test3", new PrintCommand("Test 3"));

        m_autoBuilder = new SwerveAutoBuilder(
            m_swerve::getPose,
            m_swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(0, 0, 0), //TODO: tune theses
            new PIDConstants(0, 0, 0),
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
        new JoystickButton(m_driver, XboxController.Button.kA.value).whileTrue(new SlowDrive(m_swerve));
        new JoystickButton(m_driver, XboxController.Button.kB.value).whileTrue(new OrbitPiece(m_swerve, m_lift, m_arm));
        new JoystickButton(m_driver, XboxController.Button.kY.value).onTrue(new ToggleClaw(m_air));
        new JoystickButton(m_driver, XboxController.Button.kX.value).onTrue(new ToggleTilt(m_air));
        new JoystickButton(m_driver, XboxController.Button.kStart.value).onTrue(new ZeroGyro(m_swerve));
        new JoystickButton(m_driver, XboxController.Button.kBack.value).onTrue(new ResetEncoders(m_swerve));

        new JoystickButton(m_operator, 1).whileTrue(new LockWheels(m_swerve));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        Command fullAuto = m_autoBuilder.fullAuto(m_autoChooser.getSelected());
        return fullAuto;
        // return new exampleAuto(m_swerve);
    }
}
