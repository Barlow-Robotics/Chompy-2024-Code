// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FloorIntakeConstants;
import frc.robot.Constants.ElectronicsIDs;
import com.revrobotics.REVPhysicsSim;
import org.littletonrobotics.junction.Logger;

public class FloorIntake extends SubsystemBase {

    CANSparkMax intakeMotor;  // may change to TalonFX for Kraken motors per wpk
    RelativeEncoder intakeEncoder;
    SparkPIDController intakePidController;

    boolean simulationInitialized = false;
    boolean isIntaking = false; 

    public FloorIntake() {
        intakeMotor = new CANSparkMax(ElectronicsIDs.FloorMotorID, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        configMotor(intakeMotor, false); // CHANGE - These true/false values may need to be flipped
            }

    @Override
    public void periodic() {
        logData();
    }

    public void start() {
        intakePidController.setReference(FloorIntakeConstants.MotorVelocity, ControlType.kVelocity);
        isIntaking = true;
    }

    public void stop() {
        intakePidController.setReference(0, ControlType.kVelocity);
        isIntaking = false; 
    }

    /* LOGGING */

    private void logData() {
        Logger.recordOutput("FloorIntake/ActualRPM", intakeEncoder.getVelocity());
        Logger.recordOutput("FloorIntake/IsIntaking", isIntaking);
    }

    /* CONFIG */

    private void configMotor(CANSparkMax motor, boolean inverted) {

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);

        // per Angela, this will be done in hardware
        //motor.setSmartCurrentLimit() or motor.setSecondaryCurrentLimit ?  to 40 or lower
        // motor.burnFlash(); ?  to not cause limits to default if there is a brownout

        // encoder = motor.getEncoder();
        // encoder.setVelocityConversionFactor(); // Probably don't need this :)
    
        intakePidController = intakeMotor.getPIDController();
        // set PID Controller Values
        intakePidController.setP(FloorIntakeConstants.KP);
        intakePidController.setI(FloorIntakeConstants.KI);
        intakePidController.setD(FloorIntakeConstants.KD);
        intakePidController.setIZone(FloorIntakeConstants.IZone);
        intakePidController.setFF(FloorIntakeConstants.FF);
        intakePidController.setOutputRange(-1, 1);    
    }


    /* SIMULATION */

    private void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(intakeMotor, DCMotor.getNeo550(1));
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }
    }

}
