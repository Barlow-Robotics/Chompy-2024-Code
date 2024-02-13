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
//import frc.robot.Constants.LogitechDAConstants;
import frc.robot.Constants.LogitechExtreme3DConstants;
//import frc.robot.Constants.RadioMasterConstants;
import frc.robot.commands.DriveRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        Logger.recordMetadata("ProjectName", "2024-Robot-Code"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/media/sda2/")); // Log to a USB stick
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            // CHANGE - leaks below
            /*PowerDistribution pdp =*/ new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            Logger.addDataReceiver(new WPILOGWriter(""));
            Logger.addDataReceiver(new NT4Publisher());
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        robotContainer.shooterSub.stop();
        robotContainer.floorIntakeSub.stop();

        DriverStation.silenceJoystickConnectionWarning(true);

        robotContainer.shooterMountSub.setHeightInches(Constants.ShooterMountConstants.FloorIntakeHeight);
    }

    @Override
    public void robotPeriodic() {
        robotContainer.visionSub.periodic(); 

        String currentDriverController = DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort);
        String currentOperatorController = DriverStation.getJoystickName(ElectronicsIDs.OperatorControllerPort);
        Logger.recordOutput("Controllers/Driver", currentDriverController);
        Logger.recordOutput("Controllers/Operator", currentOperatorController);

        // if (currentDriverController.equals("Logitech Extreme 3D")) {
                robotContainer.driveSub.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new DriveRobot(
                        robotContainer.driveSub, 
                        robotContainer.driverController, 
                        LogitechExtreme3DConstants.AxisX, LogitechExtreme3DConstants.AxisY, LogitechExtreme3DConstants.AxisZRotate, 
                        true));
        // } else if (DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort).equals("Radiomaster TX12 Joystick")){
        //         robotContainer.driveSub.setDefaultCommand(
        //         new DriveRobot(
        //                 robotContainer.driveSub, 
        //                 robotContainer.driverController, 
        //                 RadioMasterConstants.LeftGimbalX, RadioMasterConstants.LeftGimbalY, RadioMasterConstants.RightGimbalX, 
        //                 true));
        // } else if (DriverStation.getJoystickName(ElectronicsIDs.DriverControllerPort).equals("Logitech Dual Action")){
        //         robotContainer.driveSub.setDefaultCommand(
        //         new DriveRobot(
        //                 robotContainer.driveSub, 
        //                 robotContainer.driverController, 
        //                 LogitechDAConstants.LeftStickX, LogitechDAConstants.LeftStickY, LogitechDAConstants.RightStickX, 
        //                 true));
        // } else {
        //     System.out.println("Unknown controller");
        // }

        CommandScheduler.getInstance().run();
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
        // log auto data 
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

        /*
        frc::Field2d& debugField = vision.GetSimDebugField();
        debugField.GetObject("EstimatedRobot")->SetPose(drivetrain.GetPose());
        debugField.GetObject("EstimatedRobotModules")
            ->SetPoses(drivetrain.GetModulePoses());
            */
    }
}
