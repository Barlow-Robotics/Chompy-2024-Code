// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.Optional;
import java.util.OptionalInt;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.LogitechExtreme3DConstants;
// import frc.robot.Constants.ShooterMountConstants;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.commands.DriveRobotWithAprilTagAlign;
import frc.robot.commands.DriveRobotWithNoteAlign;
import frc.robot.commands.PivotToPoint;
import frc.robot.commands.ReverseFloorIntake;
// import frc.robot.commands.DriveRobotWithAlign;
import frc.robot.commands.SetShooterMountPosition;
import frc.robot.commands.StartClimbing;
import frc.robot.commands.StartShooterIntake;
import frc.robot.commands.StopShooterIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterMount;
import frc.robot.subsystems.ShooterMount.ShooterMountState;
import frc.robot.subsystems.Underglow;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    /* SUBSYSTEMS */

    public final Vision visionSub = new Vision();
    public final Drive driveSub = new Drive(visionSub);
    public final Shooter shooterSub = new Shooter();
    public final ShooterMount shooterMountSub = new ShooterMount(visionSub, driveSub);
    public final FloorIntake floorIntakeSub = new FloorIntake();
    public final Underglow underglowSub = new Underglow();

    /* COMMANDS */

    private final SetShooterMountPosition setShooterPosSpeakerCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.Speaker);
    private final SetShooterMountPosition setShooterPosAmpCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.Amp);
    private final SetShooterMountPosition setShooterPosSourceIntakeCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.SourceIntake);
    private final SetShooterMountPosition setShooterPosFloorCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.FloorIntake);
    private final SetShooterMountPosition setShooterPosFerryCmd = new SetShooterMountPosition(shooterMountSub,
            ShooterMountState.Ferry);

    private final StartClimbing climbCmd = new StartClimbing(shooterMountSub);
    // private final SetShooterMountPosition retryClimbCmd = new
    // SetShooterMountPosition(shooterMountSub,
    // ShooterMountState.Preclimb);
    // private final SetShooterMountPosition climbAbortCmd = new
    // 2SetShooterMountPosition(shooterMountSub,
    // ShooterMountState.ClimbAbort);

    private final StartShooterIntake startShooterIntakeCmd = new StartShooterIntake(shooterSub, floorIntakeSub,
            shooterMountSub);
    private final StopShooterIntake stopShooterIntakeCmd = new StopShooterIntake(shooterSub, floorIntakeSub);
    private final ReverseFloorIntake reverseFloorIntakeCmd = new ReverseFloorIntake(floorIntakeSub);


    private final PivotToPoint piviotToSpeakerCommand = new PivotToPoint(new Pose2d(0.5, 5.59, new Rotation2d(0)), driveSub);
    private final PivotToPoint piviotToCenterNoteCommand = new PivotToPoint(new Pose2d(2.90, 5.55, new Rotation2d(0)), driveSub);
    private final PivotToPoint piviotToAmpNoteCommand = new PivotToPoint(new Pose2d(2.90, 7, new Rotation2d(0)), driveSub);
    private final PivotToPoint piviotToStageNoteCommand = new PivotToPoint(new Pose2d(2.90, 4.10, new Rotation2d(0)), driveSub);


    /* CONTROLLERS */

    private static Joystick driverController;
    private static Joystick operatorController;

    /* BUTTONS */

    private Trigger moveToSpeakerButton; // button x
    private Trigger moveToAmpButton; // button y
    private Trigger moveToSourceButton; // left stick
    private Trigger moveToFloorButton; // left bumper
    private Trigger moveToFerryButton; // hamburger

    private Trigger climbButton; // button a
    private Trigger piviotToPoint;
    // private Trigger climbAbortButton; // right stick

    // private Trigger toggleLEDsButton; // hamburger
    // private Trigger LEDHumanSourceButton;
    // private Trigger LEDHumanFloorButton;

    private Trigger shootIntakeButton; // trigger
    private Trigger reverseFloorIntakeButton; // driver button 7

    private Trigger autoAlignButton; // driver button 11
    private Trigger restartGyroButton; // driver button 9

    private PIDController noteYawPID;
    private PIDController targetYawPID;



    /* AUTO */

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        noteYawPID = new PIDController(
                DriveConstants.YawOverrideAlignNoteKP,
                DriveConstants.YawOverrideAlignNoteKI,
                DriveConstants.YawOverrideAlignNoteKD);
        noteYawPID.setSetpoint(0.0);

        targetYawPID = new PIDController(
                DriveConstants.TargetYawOverrideAlignNoteKP,
                DriveConstants.TargetYawOverrideAlignNoteKI,
                DriveConstants.TargetYawOverrideAlignNoteKD);
        targetYawPID.setSetpoint(0.0);

        configureButtonBindings();
        driveSub.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                // new DriveRobot(
                // driveSub,
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisX),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisY),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.Slider),
                // true));

                // new DriveRobotWithAprilTagAlign(
                // driveSub,
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisX),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisY),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate),
                // () -> -driverController.getRawAxis(LogitechExtreme3DConstants.Slider),
                // true,
                // visionSub,
                // // shooterMountSub,
                // () -> autoAlignButton.getAsBoolean()));

                new DriveRobotWithNoteAlign(
                        driveSub,
                        () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisX),
                        () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisY),
                        () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate),
                        () -> -driverController.getRawAxis(LogitechExtreme3DConstants.Slider),
                        visionSub,
                        floorIntakeSub,
                        () -> autoAlignButton.getAsBoolean()));

    }

    private void configureButtonBindings() {
        driverController = new Joystick(ElectronicsIDs.DriverControllerPort);
        operatorController = new Joystick(ElectronicsIDs.OperatorControllerPort);

        /***************** DRIVE *****************/

        autoAlignButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button11);

        restartGyroButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button9);
        restartGyroButton.onTrue(new InstantCommand(() -> driveSub.zeroHeading()));

        /******************** SET SHOOTER MOUNT POSITION ********************/

        moveToSpeakerButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonX);
        moveToSpeakerButton.onTrue(setShooterPosSpeakerCmd);

        moveToAmpButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonY);
        moveToAmpButton.onTrue(setShooterPosAmpCmd);

        moveToSourceButton = new JoystickButton(operatorController, XboxControllerConstants.LeftStick);
        moveToSourceButton.onTrue(setShooterPosSourceIntakeCmd);

        moveToFloorButton = new JoystickButton(operatorController, XboxControllerConstants.LeftBumper);
        moveToFloorButton.onTrue(setShooterPosFloorCmd);

        moveToFerryButton = new JoystickButton(operatorController, XboxControllerConstants.HamburgerButton);
        moveToFerryButton.onTrue(setShooterPosFerryCmd);

        /******************** SHOOTER & INTAKE ********************/

        shootIntakeButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Trigger);
        shootIntakeButton.onTrue(startShooterIntakeCmd).onFalse(stopShooterIntakeCmd);

        reverseFloorIntakeButton = new JoystickButton(driverController, LogitechExtreme3DConstants.Button7);
        reverseFloorIntakeButton.onTrue(reverseFloorIntakeCmd).onFalse(stopShooterIntakeCmd);

        /******************** CLIMB ********************/

        climbButton = new JoystickButton(operatorController, XboxControllerConstants.ButtonA);
        climbButton.onTrue(climbCmd);

        // piviotToPoint = new JoystickButton(driverController, LogitechExtreme3DConstants.Button8);
        // piviotToPoint.onTrue(piviotToSpeakerCommand);


        // climbAbortButton = new JoystickButton(operatorController,
        // XboxControllerConstants.RightStick);
        // climbAbortButton.onTrue(retryClimbCmd);

        /********************* LED BINDINGS ************************************* */

        // LEDHumanSourceButton = new JoystickButton(operatorController,
        // XboxControllerConstants.ButtonY);
        // LEDHumanSourceButton.onTrue(new InstantCommand(() ->
        // underglowSub.LEDHumanSource = true))
        // .onFalse(new InstantCommand(() -> underglowSub.LEDHumanSource = false));

        // LEDHumanFloorButton = new JoystickButton(operatorController,
        // XboxControllerConstants.LeftTrigger);
        // LEDHumanFloorButton.onTrue(new InstantCommand(() ->
        // underglowSub.LEDHumanFloor = true))
        // .onFalse(new InstantCommand(() -> underglowSub.LEDHumanFloor = false));

        // toggleLEDsButton = new JoystickButton(operatorController,
        // XboxControllerConstants.HamburgerButton);
    }

    public void configurePathPlanner() {
        
        /* PATHPLANNER INIT */
        AutoBuilder.configureHolonomic(
                // driveSub::getPoseWithoutVision, // Robot pose supplier
                driveSub::getPose, // Robot pose supplier
                driveSub::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                driveSub::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                driveSub::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5, 0.0, 0.5), // Rotation PID constants
                        4.59, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                driveSub);

        NamedCommands.registerCommand("StartShooterIntake", startShooterIntakeCmd);
        NamedCommands.registerCommand("StopShooterIntake", stopShooterIntakeCmd);
        NamedCommands.registerCommand("SetShooterMountPositionAmp", setShooterPosAmpCmd);
        NamedCommands.registerCommand("SetShooterMountPositionSpeaker", setShooterPosSpeakerCmd);
        NamedCommands.registerCommand("SetShooterMountPositionFloor", setShooterPosFloorCmd);
        NamedCommands.registerCommand("SetRotationToSpeaker", piviotToSpeakerCommand);
        NamedCommands.registerCommand("SetRotationToCenterNote", piviotToCenterNoteCommand);


        /* SMARTDASHBOARD */

        autoChooser = AutoBuilder.buildAutoChooser(); // in order to remove autos, you must log into the roborio and
                                                      // delete them there
        SmartDashboard.putData("Selected Auto", autoChooser);
        autoChooser.setDefaultOption("BASIC", new PathPlannerAuto("BASIC"));
        Shuffleboard.getTab("Match").add("Path Name", autoChooser);

        /* LOGGING */
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

        // var startingPoseTest =
        // PathPlannerAuto.getStaringPoseFromAutoFile(autoChooser.getSelected()) ;

        PathPlannerLogging.setLogCurrentPoseCallback(
                (currentPose) -> {
                    Logger.recordOutput("Odometry/CurrentPose", currentPose);
                });
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });
    }

    public Optional<Rotation2d> getRotationTargetOverride(){

        Optional<Rotation2d> result = Optional.empty() ;

        if (RobotState.isAutonomous()) {

            if ( shooterMountSub.getShooterMountState() == ShooterMountState.FloorIntake ) {
                if (visionSub.noteIsVisible() && Math.abs(visionSub.getNoteDistanceFromCenter()) < Constants.VisionConstants.NoteAlignPixelTolerance) {
                    double yaw = visionSub.getNoteDistanceFromCenter();
                    double rotDelta = noteYawPID.calculate(yaw);
                    result =  Optional.of( driveSub.getPose().getRotation().plus(new Rotation2d(rotDelta)) ) ;
                } else {
                    noteYawPID.reset();
                }
            } else if (shooterMountSub.getShooterMountState() == ShooterMountState.Speaker && !visionSub.allDetectedTargets.isEmpty()) {
                var bestTarget = visionSub.getBestTrackableTarget() ;
                if (bestTarget.isPresent()) {
                    var rotOffset = bestTarget.get().getYaw();
                    rotOffset = targetYawPID.calculate(rotOffset);
                    result = Optional.of( driveSub.getPose().getRotation().plus(new Rotation2d(rotOffset)) ) ;
                    result = Optional.empty() ;

                    // Logger.recordOutput("YawOverrideAlign/targetYaw", bestTarget.get().getYaw());
                    // Logger.recordOutput("YawOverrideAlign/proposed rot", result.get());
                    // Logger.recordOutput("YawOverrideAlign/rot offset", result.get());
                } else {

                }
                noteYawPID.reset();

            }
        }

        return result;
    }

    public Command getAutonomousCommand() {
        if (autoChooser != null) {
            return autoChooser.getSelected();
        }
        return null;
    }
}