// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
//import org.xml.sax.ErrorHandler;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ElectronicsIDs;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    Mechanism2d shooterMountMechanism = new Mechanism2d(24, 24);
    MechanismLigament2d elevator;
    MechanismLigament2d wrist;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        Logger.recordMetadata("ProjectName", "2024-Robot-Code"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda2/")); // Log to a USB stick
            Logger.addDataReceiver(new NT4Publisher());
            // Publish data to NetworkTables
            // CHANGE - leaks below
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            Logger.addDataReceiver(new WPILOGWriter(""));
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                        // be added.

        robotContainer.shooterSub.stop();
        robotContainer.floorIntakeSub.stop();

        DriverStation.silenceJoystickConnectionWarning(true);

        robotContainer.shooterMountSub.setHeightInches(Constants.ShooterMountConstants.FloorIntakeHeight);

        MechanismRoot2d root =  shooterMountMechanism.getRoot("ShooterMount", 0, 0) ;
        elevator = root.append(new MechanismLigament2d("elevator", 2, 90, 6, new Color8Bit(Color.kWhite)));
        wrist = elevator.append(
            new MechanismLigament2d("wrist", 6, 0, 6, new Color8Bit(Color.kPurple)));

    }

    @Override
    public void robotPeriodic() {
        String currentDriverController = DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort);
        String currentOperatorController = DriverStation.getJoystickName(ElectronicsIDs.OperatorControllerPort);
        Logger.recordOutput("Controllers/Driver", currentDriverController);
        Logger.recordOutput("Controllers/Operator", currentOperatorController);

        // if (currentDriverController.equals("Logitech Extreme 3D")) {
        // robotContainer.driveSub.setDefaultCommand(
        //         // The left stick controls translation of the robot.
        //         // Turning is controlled by the X axis of the right stick.
        //         new DriveRobot(
        //                 robotContainer.driveSub,
        //                 () -> driverController.getRawAxis(LogitechExtreme3DConstants.AxisX),
        //                 () -> driverController.getRawAxis(LogitechExtreme3DConstants.AxisY),
        //                 () -> -driverController.getRawAxis(LogitechExtreme3DConstants.AxisZRotate),
        //                 true));
        // } else if
        // (DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort).equals("Radiomaster
        // TX12 Joystick")){
        // robotContainer.driveSub.setDefaultCommand(
        // new DriveRobot(
        // robotContainer.driveSub,
        // robotContainer.driverController,
        // RadioMasterConstants.LeftGimbalX, RadioMasterConstants.LeftGimbalY,
        // RadioMasterConstants.RightGimbalX,
        // true));
        // } else if
        // (DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort).equals("Logitech
        // Dual Action")){
        // robotContainer.driveSub.setDefaultCommand(
        // new DriveRobot(
        // robotContainer.driveSub,
        // robotContainer.driverController,
        // LogitechDAConstants.LeftStickX, LogitechDAConstants.LeftStickY,
        // LogitechDAConstants.RightStickX,
        // true));
        // } else {
        // System.out.println("Unknown controller");
        // }

        CommandScheduler.getInstance().run();

        elevator.setLength(2.0+robotContainer.shooterMountSub.getHeightInches()) ;
        wrist.setAngle(robotContainer.shooterMountSub.getAngleDegrees()-135);
        SmartDashboard.putData("ShooterMountMech", shooterMountMechanism);

        // Support for 3D rendering on AdvantageScope.
        Pose3d[] elevatorPoses = {
            // new Pose3d(0,0,0, new Rotation3d(0,0,0)),
            new Pose3d(0,0,0, new Rotation3d(0,0,0)),
            // new Pose3d(-0.25-0.0254,0.35+2*0.0254,-0.6-2*(0.0254), new Rotation3d(0.0,0,Math.toRadians(90))),
            new Pose3d(0,0,(robotContainer.shooterMountSub.getHeightInches()/2)*0.0254, new Rotation3d(0,0,0)),
            new Pose3d(0,0,(robotContainer.shooterMountSub.getHeightInches())*0.0254, new Rotation3d(0,0,0)),
            // new Pose3d(0,0,(robotContainer.shooterMountSub.getHeightInches())*0.0254, 
            //     new Rotation3d(0,Math.toRadians(robotContainer.shooterMountSub.getAngleDegrees()-135),0))
            new Pose3d(0,0,0, 
                new Rotation3d(0,45,0))
                // new Rotation3d(0,Math.toRadians(robotContainer.shooterMountSub.getAngleDegrees()-135),0))
            // new Pose3d(-0.25-0.0254,0.35+2*0.0254,-0.6-2*(0.0254), new Rotation3d(0.0,0,Math.toRadians(90)))
        } ;

        Logger.recordOutput("ShooterMount/poses", elevatorPoses);
    }
    

    @Override
    public void disabledInit() {
        robotContainer.driveSub.stop();
        robotContainer.floorIntakeSub.stop();
        robotContainer.shooterSub.stop();
        robotContainer.shooterMountSub.stop();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
        robotContainer.driveSub.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().run();

        var simPose = robotContainer.driveSub.getPoseWithoutVision();

        robotContainer.visionSub.simulationPeriodic(simPose);

        double elevatorTop = robotContainer.shooterMountSub.getHeightInches()*0.0254 ;
        double shooterPitchAngle = -robotContainer.shooterMountSub.getAngleDegrees()+65 ;

        // Support for 3D rendering on AdvantageScope.
        Pose3d[] elevatorPoses = {
            // new Pose3d(0,0,0, new Rotation3d(0,0,0)),
            //base stage
            new Pose3d(0,0,0, new Rotation3d(0,0,0)),
            // first stage
            new Pose3d(0,0,elevatorTop/2.0, new Rotation3d(0,0,0)),
            // second stage
            new Pose3d(0,0,elevatorTop, new Rotation3d(0,0,0)),
            // shooter
            new Pose3d(8*0.0254,0,22*0.0254 + elevatorTop , new Rotation3d(0,Math.toRadians(shooterPitchAngle),0)),
            // buympers
            new Pose3d(0,0,0, new Rotation3d(0,0,0))
        } ;

        Logger.recordOutput("ShooterMount/poses", elevatorPoses);
        Logger.recordOutput("ShooterMount/pitchAngle", shooterPitchAngle);
        //wpk


        /*
         * frc::Field2d& debugField = vision.GetSimDebugField();
         * debugField.GetObject("EstimatedRobot")->SetPose(drivetrain.GetPose());
         * debugField.GetObject("EstimatedRobotModules")
         * ->SetPoses(drivetrain.GetModulePoses());
         */
    }
}
