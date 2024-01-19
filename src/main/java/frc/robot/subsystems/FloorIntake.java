// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class FloorIntake extends SubsystemBase {

  private static final int upperMotorID = 0;
  private static final int lowerMotorID = 0;

  private static final int upperEncoderID = 0;
  private static final int lowerEncoderID = 0;
  
  CANSparkMax upperMotor;
  CANcoder upperEncoder;

  CANSparkMax lowerMotor;
  CANcoder lowerEncoder;

  public FloorIntake(double upperMagnetOffset, double lowerMagnetOffset) {
        upperMotor = new CANSparkMax(upperMotorID, MotorType.kBrushless);
        upperMotor.restoreFactoryDefaults();
        upperMotor.setIdleMode(IdleMode.kBrake);
        upperMotor.setInverted(true);

        upperEncoder = new CANcoder(upperEncoderID, "rio");
        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();

        MagnetSensorConfigs upperMagnetConfig = new MagnetSensorConfigs();
        upperMagnetConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        if (!Robot.isSimulation()) upperMagnetConfig.MagnetOffset = upperMagnetOffset;
        else upperMagnetConfig.MagnetOffset = 0.0;
        
        upperMagnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        canCoderConfiguration.MagnetSensor = upperMagnetConfig;

        // need to be added
        // canCoderConfiguration.initializationStrategy =
        // SensorInitializationStrategy.BootToAbsolutePosition; // BW sets sensor to be absolute zero
        // canCoderConfiguration.sensorCoefficient = Math.PI / 2048.0;

        upperEncoder.getConfigurator().apply(canCoderConfiguration);


        lowerMotor = new CANSparkMax(lowerMotorID, MotorType.kBrushless);
        lowerMotor.restoreFactoryDefaults();
        lowerMotor.setIdleMode(IdleMode.kBrake);
        lowerMotor.setInverted(true);

        lowerEncoder = new CANcoder(lowerEncoderID, "rio");

        MagnetSensorConfigs lowerMagnetConfig = new MagnetSensorConfigs();
        lowerMagnetConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        if (!Robot.isSimulation()) lowerMagnetConfig.MagnetOffset = lowerMagnetOffset;
        else lowerMagnetConfig.MagnetOffset = 0.0;
        
        lowerMagnetConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        canCoderConfiguration.MagnetSensor = lowerMagnetConfig;

        // need to be added
        // canCoderConfiguration.initializationStrategy =
        // SensorInitializationStrategy.BootToAbsolutePosition; // BW sets sensor to be absolute zero
        // canCoderConfiguration.sensorCoefficient = Math.PI / 2048.0;

        lowerEncoder.getConfigurator().apply(canCoderConfiguration);

  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  

}
