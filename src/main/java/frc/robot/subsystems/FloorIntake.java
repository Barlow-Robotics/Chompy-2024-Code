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

    CANSparkMax intakeMotor;
    RelativeEncoder intakeEncoder;
    SparkPIDController intakePidController;

    boolean simulationInitialized = false;
    boolean isIntaking = false; 

    public FloorIntake() {
        intakeMotor = new CANSparkMax(ElectronicsIDs.FloorMotorID, MotorType.kBrushless);
        intakeEncoder = intakeMotor.getEncoder();
        motorAndEncoderConfig(intakeMotor, intakeEncoder, false); // CHANGE - These true/false values may need to be flipped
        intakePidController = intakeMotor.getPIDController();
        setPIDControllerValues(
                intakePidController,
                FloorIntakeConstants.KP,
                FloorIntakeConstants.KI,
                FloorIntakeConstants.KD,
                FloorIntakeConstants.IZone,
                FloorIntakeConstants.FF);
    }

    @Override
    public void periodic() {
        advantageKitLogging();
    }

    public void startIntaking() {
        intakePidController.setReference(FloorIntakeConstants.MotorVelocity, ControlType.kVelocity);
        isIntaking = true;
    }

    public void stopIntaking() {
        intakePidController.setReference(0, ControlType.kVelocity);
        isIntaking = false; 
    }

    /* LOGGING */

    private void advantageKitLogging() {
        Logger.recordOutput("FloorIntake/ActualRPM", intakeEncoder.getVelocity());
        Logger.recordOutput("FloorIntake/IsIntaking", isIntaking);
    }

    /* CONFIG */

    private void motorAndEncoderConfig(CANSparkMax motor, RelativeEncoder encoder, boolean inverted) {

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);

        encoder = motor.getEncoder();
        // encoder.setVelocityConversionFactor(); // Probably don't need this :)
    }

    private void setPIDControllerValues(SparkPIDController controller, double kP, double kI, double kD, double kIz, double kFF) {
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
        controller.setIZone(kIz);
        controller.setFF(kFF);
        controller.setOutputRange(-1, 1);
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
