// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FloorIntakeConstants;
import frc.robot.Constants.ElectronicIDs;
import com.revrobotics.REVPhysicsSim;

public class FloorIntake extends SubsystemBase {

    // temp constants - move to constants file?
    boolean simulationInitialized = false;
    private static final int simulationVelocity = 6800; // CHANGE
    private static final double simulationTime = 0.5; // CHANGE

    CANSparkMax upperMotor;
    RelativeEncoder upperEncoder;
    SparkPIDController upperPidController;

    CANSparkMax lowerMotor;
    RelativeEncoder lowerEncoder;
    SparkPIDController lowerPidController;

    DigitalInput breakBeam;

    public FloorIntake() {

        /* UPPER MOTOR CONFIG */
        upperMotor = new CANSparkMax(ElectronicIDs.upperFloorMotorID, MotorType.kBrushless);
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
        lowerMotor = new CANSparkMax(ElectronicIDs.lowerFloorMotorID, MotorType.kBrushless);
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

        breakBeam = new DigitalInput(ElectronicIDs.floorBreakBeamID);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void startIntaking() {
        upperMotor.set(FloorIntakeConstants.MotorSpeed);
        lowerMotor.set(FloorIntakeConstants.MotorSpeed);
    }

    public void stopIntaking() {
        upperMotor.set(0);
        lowerMotor.set(0);
    }

    public double getRPMUpper() {
        return upperEncoder.getVelocity();
    }

    public double getRPMLower() {
        return lowerEncoder.getVelocity();
    }

    public boolean isIntaking() {
        if (((0.95 * FloorIntakeConstants.MotorSpeed) <= getRPMUpper()) && ((0.95 * FloorIntakeConstants.MotorSpeed) <= getRPMLower())) {
            return true; 
        } else if ((0.95 * FloorIntakeConstants.MotorSpeed) <= getRPMUpper()) {
            System.out.println("Upper motor isn't up to speed");
            return false;
        } else {
            System.out.println("Lower motor isn't up to speed");
            return false;
        }
    }

    public boolean noteLoaded() {
        return breakBeam.get(); // May need to CHANGE
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

    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("RPM Upper", this::getRPMUpper, null);
        builder.addDoubleProperty("RPM Lower", this::getRPMLower, null);
        builder.addBooleanProperty("Floor Intaking", this::isIntaking, null);
        builder.addBooleanProperty("Note Loaded", this::noteLoaded, null);

        // builder.addStringProperty()
    }

    // Simulation Code
    public void simulationInit() {
        REVPhysicsSim.getInstance().addSparkMax(upperMotor, DCMotor.getNeo550(1));
        REVPhysicsSim.getInstance().addSparkMax(lowerMotor, DCMotor.getNeo550(1));
        //LT -  add one for DigitalInput breakBeam ?
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }
    }

}
