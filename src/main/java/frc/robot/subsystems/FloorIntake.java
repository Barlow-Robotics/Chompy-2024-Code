// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class FloorIntake extends SubsystemBase {

    /*********************************************************************/
    /***************************** CONSTANTS *****************************/

    // upper = 1_ | lower = 2_

    private static final int upperMotorID = 11;
    private static final int lowerMotorID = 21;

    private static final int upperEncoderID = 11;
    private static final int lowerEncoderID = 21;
    
    private static final int upperPIDControllerID = 11;
    private static final int lowerPIDControllerID = 21;

    /* UPPER PID CONTROLLER */
    private static final double UpperKP = 0.5; // CHANGE
    private static final double UpperKI = 0; // CHANGE
    private static final double UpperKD = 0; // CHANGE
    private static final double UpperIZone = 0; // CHANGE
    private static final double UpperFF = 1; // CHANGE

    /* LOWER PID CONTROLLER */
    private static final double LowerKP = 0.5; // CHANGE
    private static final double LowerKI = 0; // CHANGE
    private static final double LowerKD = 0; // CHANGE
    private static final double LowerIZone = 0; // CHANGE
    private static final double LowerFF = 1; // CHANGE

    private static final double motorSpeed = 0; // CHANGE

    private static final int breakBeamID = 0; // CHANGE

    /*******************************************************************************/
    /*******************************************************************************/

    CANSparkMax upperMotor;
    RelativeEncoder upperEncoder;
    SparkPIDController upperPidController;

    CANSparkMax lowerMotor;
    RelativeEncoder lowerEncoder;
    SparkPIDController lowerPidController;

    DigitalInput breakBeam;

    public FloorIntake(double upperMagnetOffset, double lowerMagnetOffset) {

        /* MOTOR CONFIG STUFF */
        upperMotor = new CANSparkMax(upperMotorID, MotorType.kBrushless);
        upperEncoder = upperMotor.getEncoder();
        motorAndEncoderConfig(upperMotor, upperEncoder, upperMagnetOffset, false); // CHANGE - These true/false values may need to be flipped
        upperPidController = upperMotor.getPIDController();
        setPIDControllerValues(upperPidController, UpperKP, UpperKI, UpperKD, UpperIZone, UpperFF);

        lowerMotor = new CANSparkMax(lowerMotorID, MotorType.kBrushless);
        lowerEncoder = lowerMotor.getEncoder();
        motorAndEncoderConfig(lowerMotor, lowerEncoder, lowerMagnetOffset, true); // CHANGE - These true/false values may need to be flipped
        lowerPidController = lowerMotor.getPIDController();
        setPIDControllerValues(lowerPidController, LowerKP, LowerKI, LowerKD, LowerIZone, LowerFF);


        breakBeam = new DigitalInput(breakBeamID);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void startIntaking() {   
        upperMotor.set(motorSpeed);
        lowerMotor.set(motorSpeed);
    }

    public void stopIntaking() {
        upperMotor.set(0);
        lowerMotor.set(0);
    }

    public double getRPM() {
        return upperEncoder.getVelocity();
    }

    public boolean isIntaking() {
        return (0.95*motorSpeed) <= getRPM();
    }

    public boolean noteLoaded() {
        return breakBeam.get(); // May need to CHANGE
    } 

    private void motorAndEncoderConfig(CANSparkMax motor, RelativeEncoder encoder, double magnetOffset, boolean inverted) {
       
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

}
