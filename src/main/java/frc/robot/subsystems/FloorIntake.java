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
import frc.robot.Constants;
import frc.robot.Constants.ElectronicIDs;
import com.revrobotics.REVPhysicsSim;
import org.littletonrobotics.junction.Logger;

public class FloorIntake extends SubsystemBase {

    CANSparkMax upperMotor;
    RelativeEncoder upperEncoder;
    SparkPIDController upperPidController;

    CANSparkMax lowerMotor;
    RelativeEncoder lowerEncoder;
    SparkPIDController lowerPidController;

    boolean simulationInitialized = false;
    boolean isIntaking = false; 

    public FloorIntake() {

        /* UPPER MOTOR CONFIG */
        upperMotor = new CANSparkMax(ElectronicIDs.UpperFloorMotorID, MotorType.kBrushless);
        upperEncoder = upperMotor.getEncoder();
        motorAndEncoderConfig(upperMotor, upperEncoder, false); // CHANGE - These true/false values may need to be flipped
        upperPidController = upperMotor.getPIDController();
        setPIDControllerValues(
                upperPidController,
                FloorIntakeConstants.UpperKP,
                FloorIntakeConstants.UpperKI,
                FloorIntakeConstants.UpperKD,
                FloorIntakeConstants.UpperIZone,
                FloorIntakeConstants.UpperFF);

        /* LOWER MOTOR CONFIG */
        lowerMotor = new CANSparkMax(ElectronicIDs.LowerFloorMotorID, MotorType.kBrushless);
        lowerEncoder = lowerMotor.getEncoder();
        motorAndEncoderConfig(lowerMotor, lowerEncoder, true); // CHANGE - These true/false values may need to be flipped
        lowerPidController = lowerMotor.getPIDController();
        setPIDControllerValues(
                lowerPidController,
                FloorIntakeConstants.LowerKP,
                FloorIntakeConstants.LowerKI,
                FloorIntakeConstants.LowerKD,
                FloorIntakeConstants.LowerIZone,
                FloorIntakeConstants.LowerFF);
    }

    @Override
    public void periodic() {
        advantageKitLogging();
    }

    public void startIntaking() {
        upperPidController.setReference(FloorIntakeConstants.MotorVelocity, ControlType.kVelocity);
        lowerPidController.setReference(FloorIntakeConstants.MotorVelocity, ControlType.kVelocity);
        isIntaking = true;
    }

    public void stopIntaking() {
        upperPidController.setReference(0, ControlType.kVelocity);
        lowerPidController.setReference(0, ControlType.kVelocity);
        isIntaking = false; 
    }

    /* LOGGING */

    private void advantageKitLogging() {
        Logger.recordOutput("FloorIntake/ActualRPMLower", lowerEncoder.getVelocity());
        Logger.recordOutput("FloorIntake/ActualRPMUpper", upperEncoder.getVelocity());
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
        REVPhysicsSim.getInstance().addSparkMax(upperMotor, DCMotor.getNeo550(1));
        REVPhysicsSim.getInstance().addSparkMax(lowerMotor, DCMotor.getNeo550(1));
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }
    }

}
