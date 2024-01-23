 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveRobot;
import frc.robot.commands.StartFloorIntake;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopFloorIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class RobotContainer {

    /********************************************************************/
    /***** CONSTANTS *****/

    public static final int LDALeftStickX = 0; // LDA = Logitech Dual Action
    public static final int LDALeftStickY = 1;
    public static final int LDARightStickX = 2;
    public static final int LDARightStickY = 3;
    public static final int LDALeftTrigger = 7; // Speaker
    public static final int LDARightTrigger = 8; // Amp
    public static final int LDAButtonA = 2; 
    public static final int LDAButtonB = 3; // Trapdoor
    public static final int LDAButtonX = 1;
    public static final int LDAButtonY = 4;
    public static final int LDALeftBumper = 5; // Floor Intake
    public static final int LDARightBumper = 6; // Source Intake
    public static final int LDABackButton = 9;
    public static final int LDAStartButton = 10;
    public static final int LDALeftStick = 11;
    public static final int LDARightStick = 12;
    public static final double LDAForwardAxisAttenuation = -0.5;
    public static final double LDALateralAxisAttenuation = 0.5;
    public static final double LDAYawAxisAttenuation = 0.5;

    /********************************************************************/
    /********************************************************************/

    /* SUBSYSTEMS */
    public final Drive driveSub = new Drive();
    public final Shooter shooterSub = new Shooter();
    public final FloorIntake floorIntakeSub = new FloorIntake();

    /* COMMANDS */
    private final StartShooter startShooterSpeakerCmd = new StartShooter(shooterSub, shooterSub.shooterState.Speaker);
    private final StartShooter startShooterAmpCmd = new StartShooter(shooterSub, shooterSub.shooterState.Amp);
    private final StartShooter startShooterSourceIntakeCmd = new StartShooter(shooterSub, shooterSub.shooterState.Source);
    private final StartShooter startShooterFloorIntakeCmd = new StartShooter(shooterSub, shooterSub.shooterState.Chassis);
    private final StartShooter startShooterTrapCmd = new StartShooter(shooterSub, shooterSub.shooterState.Trapdoor);
    
    private final StartFloorIntake startFloorIntakeCmd = new StartFloorIntake(floorIntakeSub);
    private final StopFloorIntake stopFloorIntakeCmd = new StopFloorIntake(floorIntakeSub);
    
    /* CONTROLLERS */
    PS4Controller driverController; 
    Joystick operatorController;
    
    /* BUTTONS */
   
    public Trigger shootButtonSpeaker;
    public Trigger shootButtonAmp;
    public Trigger shootButtonSourceIntake;
    public Trigger shootButtonFloorIntake;
    public Trigger shootButtonTrap;

    // private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        configureBindings();

        driveSub.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new DriveRobot(
                        driveSub, driverController, LDALeftStickY, LDALeftStickX, LDARightStickX, true));

        AutoBuilder.configureHolonomic(
                driveSub::getPose, // Robot pose supplier
                driveSub::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                driveSub::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                driveSub::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig(
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
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
                driveSub
        );

                NamedCommands.registerCommand("LeaveArea", Commands.print("***********************************Left Area"));
        NamedCommands.registerCommand("ScoreInAmp", Commands.print("*******************************Shot in amp"));


        NamedCommands.registerCommand("LeaveArea", Commands.print("**********************************Left Area"));
        NamedCommands.registerCommand("ScoreInAmp", Commands.runOnce(driveSub::scoreAmp, driveSub));

        // NamedCommands.registerCommand("marker3", Commands.print("Passed marker 3"));
        // NamedCommands.registerCommand("marker4", Commands.print("Passed marker 4"));

        // autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
       // SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {

        driverController = new PS4Controller(1);
        operatorController = new Joystick(2);
        
        shootButtonSpeaker = new JoystickButton(operatorController, LDALeftTrigger);
        shootButtonSpeaker.onTrue(startShooterSpeakerCmd);
        shootButtonAmp = new JoystickButton(operatorController, LDARightTrigger);
        shootButtonAmp.onTrue(startShooterAmpCmd);
        shootButtonSourceIntake = new JoystickButton(operatorController, LDARightBumper);
        shootButtonSourceIntake.onTrue(startShooterSourceIntakeCmd);
        shootButtonFloorIntake = new JoystickButton(operatorController, LDALeftBumper);
        shootButtonFloorIntake.onTrue(startShooterFloorIntakeCmd);
        shootButtonTrap = new JoystickButton(operatorController, LDAButtonB);
        shootButtonTrap.onTrue(startShooterTrapCmd);        
        // Add a button to run the example auto to SmartDashboard, this will also be in
        // the auto chooser built above
        // SmartDashboard.putData("Example Auto", new PathPlannerAuto("test"));

        // Add a button to run pathfinding commands to SmartDashboard
        // SmartDashboard.putData("Pathfind to Pickup Pos", 
        //     AutoBuilder.pathfindToPose(
        //         new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)),
        //         new PathConstraints(
        //                 4.0, 4.0,
        //                 Units.degreesToRadians(360), Units.degreesToRadians(540)),
        //         0,
        //         2.0));
        
        // SmartDashboard.putData("Pathfind to Scoring Pos", 
        //     AutoBuilder.pathfindToPose(
        //         new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
        //         new PathConstraints(
        //                 4.0, 4.0,
        //                 Units.degreesToRadians(360), Units.degreesToRadians(540)),
        //         0,
        //         0));

        // Add a button to SmartDashboard that will create and follow an on-the-fly path
        // This example will simply move the robot 2m forward of its current position
        // SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
        //     Pose2d currentPose = driveSub.getPose();

        //     // The rotation component in these poses represents the direction of travel
        //     Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        //     Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)),
        //             new Rotation2d());

        //     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
        //     PathPlannerPath path = new PathPlannerPath(
        //             bezierPoints,
        //             new PathConstraints(
        //                     4.0, 4.0,
        //                     Units.degreesToRadians(360), Units.degreesToRadians(540)),
        //             new GoalEndState(0.0, currentPose.getRotation()));

        //     AutoBuilder.followPathWithEvents(path).schedule();
        // }));
    }

    
    
    public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        // return null;
        // return AutoBuilder.pathfindToPose(
        //     new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
        //     new PathConstraints(
        //       1.0, 1.0, 
        //       Units.degreesToRadians(360), Units.degreesToRadians(540)
        //     ), 
        //     0, 
        //     2.0
        //   );
        return new PathPlannerAuto("Score Amp");
    }
}