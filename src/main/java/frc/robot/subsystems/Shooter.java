// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sim.PhysicsSim;

public class Shooter extends SubsystemBase {
    private final TalonFX lowerShooterMotor;
    private final TalonFXSimState lowerShooterMotorSim;
    private final DCMotorSim lowerMotorModel = 
        new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1, ShooterConstants.jKgMetersSquared);
    
    private final TalonFX upperShooterMotor;
    private final TalonFXSimState upperShooterMotorSim;
    private final DCMotorSim upperMotorModel = 
        new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1, ShooterConstants.jKgMetersSquared);

    private final CANSparkMax indexMotor;
    private final RelativeEncoder indexEncoder;
    private final SparkPIDController indexPidController;
    
    private final VelocityVoltage voltageVelocity = 
        new VelocityVoltage(0, 0, true, 0, 0, 
                            false, false, false);
    private final NeutralOut brake = new NeutralOut();

    DigitalInput breakBeam;
    DIOSim breakBeamSim;

    private boolean simulationInitialized = false;
    private boolean isIndexing = false;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    private GenericEntry shuffleBoardSpeed = tab.add("ShuffleBoard Speed", 1).getEntry();

    public Shooter() {
        lowerShooterMotor = new TalonFX(ElectronicsIDs.LowerShooterMotorID); 
        upperShooterMotor = new TalonFX(ElectronicsIDs.UpperShooterMotorID); 
        
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
       
        setTalonConfigs(talonConfigs);
        applyMotorConfigs(
            lowerShooterMotor, "lowerShooterMotor", 
            talonConfigs, motorOutputConfigs, InvertedValue.CounterClockwise_Positive);
        applyMotorConfigs(
            upperShooterMotor, "upperShooterMotor", 
            talonConfigs, motorOutputConfigs, InvertedValue.Clockwise_Positive);

        lowerShooterMotorSim = lowerShooterMotor.getSimState();
        upperShooterMotorSim = upperShooterMotor.getSimState();

        indexMotor = new CANSparkMax(ElectronicsIDs.IndexMotorID, MotorType.kBrushless);
        indexEncoder = indexMotor.getEncoder();
        motorAndEncoderConfig(indexMotor, indexEncoder, false); // CHANGE - These true/false values may need to be flipped
        indexPidController = indexMotor.getPIDController();
        setPIDControllerValues(
                indexPidController,
                ShooterConstants.IndexKP,
                ShooterConstants.IndexKI,
                ShooterConstants.IndexKD,
                ShooterConstants.IndexIZone,
                ShooterConstants.IndexFF);

        breakBeam = new DigitalInput(ElectronicsIDs.BreakBeamID);
    }

    @Override
    public void periodic() {
        advantageKitLogging();
    }

    public void setVelocity(double shooterRPM, double indexRPM) {
        // if (shuffleBoardSpeed.getDouble(-1.0) <= Constants.Falcon500MaxRPM / 60) { // max rps: 105
        //         shooterRPM = shuffleBoardSpeed.getDouble(-1.0);
        // }
        Logger.recordOutput("Shooter/DesiredShooterRPM", shooterRPM);
        Logger.recordOutput("Shooter/DesiredIndexRPM", indexRPM);

        shooterRPM /= 60;

        lowerShooterMotor.setControl(voltageVelocity.withVelocity(shooterRPM));
        upperShooterMotor.setControl(voltageVelocity.withVelocity(shooterRPM));

        if (isWithinVelocityTolerance(shooterRPM)) {
            indexPidController.setReference(indexRPM, ControlType.kVelocity);
            isIndexing = true;
        }       
    }

    public void stopShooting() {
        lowerShooterMotor.setControl(brake);
        upperShooterMotor.setControl(brake);
        indexPidController.setReference(0, ControlType.kVelocity);
        isIndexing = false;
    }

    private double getRPM(TalonFX motor) {
        return motor.getVelocity().getValue()*Constants.SecondsPerMinute;
    }

    private boolean isWithinVelocityTolerance(double desiredRPM) {
        return (getRPM(lowerShooterMotor) >= Constants.LowerToleranceLimit * desiredRPM) &&
                (getRPM(lowerShooterMotor) <= Constants.UpperToleranceLimit * desiredRPM) &&
                (getRPM(upperShooterMotor) >= Constants.LowerToleranceLimit * desiredRPM) &&
                (getRPM(upperShooterMotor) <= Constants.UpperToleranceLimit * desiredRPM);
    }

    public boolean isNoteLoaded() {
        return breakBeam.get();
    }

    /* LOGGING */

    private void advantageKitLogging() {
        Logger.recordOutput("Shooter/ActualRPMLower", getRPM(lowerShooterMotor));
        Logger.recordOutput("Shooter/ActualRPMUpper", getRPM(upperShooterMotor));
        Logger.recordOutput("Shooter/IsIndexing", isIndexing);
        Logger.recordOutput("Shooter/ClosedLoopErrorLower", lowerShooterMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Shooter/ClosedLoopErrorUpper", upperShooterMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Shooter/IsShootingAmp", isWithinVelocityTolerance(ShooterConstants.AmpRPM));
        Logger.recordOutput("Shooter/IsShootingSpeaker", isWithinVelocityTolerance(ShooterConstants.SpeakerRPM));
        Logger.recordOutput("Shooter/IsShootingTrap", isWithinVelocityTolerance(ShooterConstants.TrapRPM));
        Logger.recordOutput("Shooter/IsIntakingSource", isWithinVelocityTolerance(ShooterConstants.SourceRPM));
        Logger.recordOutput("Shooter/IsIntakingFloor", isWithinVelocityTolerance(ShooterConstants.FloorRPM));
        Logger.recordOutput("Shooter/IsNoteLoaded", isNoteLoaded());
    }

    /* CONFIG */

    private void setTalonConfigs(TalonFXConfiguration configs) {
        configs.Slot0.kP = ShooterConstants.ShooterKP;
        // configs.Slot0.kI = ShooterConstants.ShooterKI;
        // configs.Slot0.kD = ShooterConstants.ShooterKD;
        configs.Slot0.kV = ShooterConstants.ShooterKV;
        configs.Voltage.PeakForwardVoltage = ShooterConstants.PeakShooterForwardVoltage; // Peak output of 8 volts
        configs.Voltage.PeakReverseVoltage = ShooterConstants.PeakShooterReverseVoltage;
    }

    private void applyMotorConfigs(
        TalonFX motor, String motorName, 
        TalonFXConfiguration talonConfigs, MotorOutputConfigs motorOutputConfigs, InvertedValue inversion) {
        
        motorOutputConfigs.Inverted = inversion;
        
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        // attempt setting configs up to 5 times
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(talonConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply talon configs to " + motorName + " error code: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply motor output configs to " + motorName + " error code: " + status.toString());
        }
    }

    private void motorAndEncoderConfig(CANSparkMax motor, RelativeEncoder encoder, boolean inverted) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);

        encoder = motor.getEncoder();
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

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(lowerShooterMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(upperShooterMotor, 0.001);
        REVPhysicsSim.getInstance().addSparkMax(indexMotor, DCMotor.getNeo550(1));

        lowerShooterMotorSim.Orientation = ChassisReference.CounterClockwise_Positive; // CHANGE
        upperShooterMotorSim.Orientation = ChassisReference.Clockwise_Positive; // CHANGE

        breakBeamSim = new DIOSim(breakBeam);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        lowerShooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double lowerVoltage = lowerShooterMotorSim.getMotorVoltage();
        lowerMotorModel.setInputVoltage(lowerVoltage);
        lowerMotorModel.update(0.02);
        lowerShooterMotorSim.setRotorVelocity(lowerMotorModel.getAngularVelocityRPM() / 60.0);
        lowerShooterMotorSim.setRawRotorPosition(lowerMotorModel.getAngularPositionRotations());

        upperShooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double upperVoltage = upperShooterMotorSim.getMotorVoltage();
        upperMotorModel.setInputVoltage(upperVoltage);
        upperMotorModel.update(0.02);
        upperShooterMotorSim.setRotorVelocity(upperMotorModel.getAngularVelocityRPM() / 60.0);
        upperShooterMotorSim.setRawRotorPosition(upperMotorModel.getAngularPositionRotations());
    }

}
