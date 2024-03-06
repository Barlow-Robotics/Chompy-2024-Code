// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectronicsIDs;
import frc.robot.Constants.ShooterMountConstants;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;

import frc.robot.Constants;

public class ShooterMount extends SubsystemBase {

    TalonFX angleMotor;
    private final TalonFXSimState angleMotorSim;
    private final DCMotorSim angleMotorModel = new DCMotorSim(edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1),
            1, Constants.jKgMetersSquared);

    TalonFX leftElevatorMotor;
    private final TalonFXSimState leftElevatorMotorSim;
    private final DCMotorSim leftElevatorMotorModel = new DCMotorSim(
            edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1, Constants.jKgMetersSquared);

    TalonFX rightElevatorMotor;
    private final TalonFXSimState rightElevatorMotorSim;
    private final DCMotorSim rightElevatorMotorModel = new DCMotorSim(
            edu.wpi.first.math.system.plant.DCMotor.getFalcon500(1), 1, Constants.jKgMetersSquared);

    DigitalInput bottomHallEffect;
    DIOSim bottomHallEffectSim;

    private final CANcoder angleCANCoder; // needs an encoder
    private final CANcoderSimState absoluteAngleEncoderSim; // CHANGE needed? never used

    public enum ShooterMountState {
        Speaker, Amp, SourceIntake, FloorIntake, Climb, MovingToPosition, ClimbAbort, Ferry
    }

    private ShooterMountState shooterPosState = ShooterMountState.FloorIntake;

    private boolean simulationInitialized = false;

    private final Drive driveSub ;
    private final Vision visionSub ;
    private double desiredDegrees = Constants.ShooterMountConstants.FloorIntakeAngle ;

    public ShooterMount( Vision visionSub, Drive driveSub ) {
        bottomHallEffect = new DigitalInput(ElectronicsIDs.BottomHallEffectID);

        angleMotor = new TalonFX(ElectronicsIDs.AngleMotorID);
        angleMotorSim = angleMotor.getSimState();

        leftElevatorMotor = new TalonFX(ElectronicsIDs.LeftElevatorMotorID);
        leftElevatorMotorSim = leftElevatorMotor.getSimState();
        leftElevatorMotor.setPosition(0);

        rightElevatorMotor = new TalonFX(ElectronicsIDs.RightElevatorMotorID);
        rightElevatorMotorSim = rightElevatorMotor.getSimState();

        angleCANCoder = new CANcoder(ElectronicsIDs.AngleEncoderID, "rio");
        absoluteAngleEncoderSim = angleCANCoder.getSimState();

        applyAngleMotorConfigs(InvertedValue.Clockwise_Positive);
        applyAngleEncoderConfigs();
        applyElevatorMotorConfigs(leftElevatorMotor, "leftElevatorMotor", InvertedValue.CounterClockwise_Positive);
        applyElevatorMotorConfigs(rightElevatorMotor, "rightElevatorMotor", InvertedValue.Clockwise_Positive); // don't need to do this for right since follower, just doing it for the current limit 
        setNeutralMode(NeutralModeValue.Brake, NeutralModeValue.Brake);

        rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));

        this.driveSub = driveSub ;
        this.visionSub = visionSub ;
    }

    @Override
    public void periodic() {
        logData();

        if ((isAtBottom() && leftElevatorMotor.getVelocity().getValue() < 0) ||
                (getHeightInches() == ShooterMountConstants.MaxHeightInches
                        && leftElevatorMotor.getVelocity().getValue() > 0)) {
            stopElevatorMotor();
        }

        if ((getAngleCANCoderDegrees() == ShooterMountConstants.MinAngleDegrees
                && angleMotor.getVelocity().getValue() < 0) ||
                (getHeightInches() == ShooterMountConstants.MaxAngleDegrees
                        && angleMotor.getVelocity().getValue() > 0)) {
            stopAngleMotor();
        }

        // // wpk remove return statement to enable this code
        // //return ;
        // double trackedAngle = 0.0 ;
        //     // trackedAngle = getSpeakerShooterAngle() ;  //wpk temp. remove this line later
        // // Logger.recordOutput("ShooterMount/Angle/TrackedAngle", trackedAngle);
        // if ( this.shooterPosState == ShooterMountState.Speaker) {
        //     trackedAngle = getSpeakerShooterAngle() ;
        //     //trackedAngle = this.desiredDegrees ;
        // } else {
        //     trackedAngle = this.desiredDegrees ;
        // }
        // Logger.recordOutput("ShooterMount/Angle/TrackedAngle", trackedAngle);

        // setAngle(trackedAngle);

        // Logger.recordOutput("ShooterMount/Angle/NewIsWithinAngleTolerance", isWithinAngleTolerance2());



    }

    /* ANGLE */

    public void setAngle(double desiredDegrees) {

        final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(desiredDegrees));
        angleMotor.setControl(request);
        this.desiredDegrees = desiredDegrees ;

        // wpk test code
        // double newAngle = 0.0 ;
        // if (this.shooterPosState == ShooterMountState.Speaker) {
        //     newAngle = getSpeakerShooterAngle() ;
        // } else {
        //     newAngle = desiredDegrees ;
        // }       
        // final MotionMagicVoltage request = new MotionMagicVoltage(Units.degreesToRotations(newAngle));
        // angleMotor.setControl(request);
        // this.desiredDegrees = newAngle ;
    }

    public void stopAngleMotor() {
        angleMotor.set(0);
    }

    public void setAngleWithVision() {
        // Need to make this
    }

    public double getAngleCANCoderDegrees() {
        return Units.rotationsToDegrees(angleCANCoder.getAbsolutePosition().getValue());
    }

    private double getTalonEncoderDegrees() {
        return Units.rotationsToDegrees(angleMotor.getPosition().getValue());
    }

    /* ELEVATOR */

    public void setHeightInches(double desiredInches) {
        double rotations = ((desiredInches - ShooterMountConstants.StartingHeight) / 2)
                * ShooterMountConstants.RotationsPerElevatorInch;
        MotionMagicVoltage request = new MotionMagicVoltage(rotations);
        leftElevatorMotor.setControl(request);
    }

    public double getHeightInches() {
        double elevatorHeight = ((leftElevatorMotor.getPosition().getValue()
                / ShooterMountConstants.RotationsPerElevatorInch) * 2)
                + ShooterMountConstants.StartingHeight;
        return elevatorHeight;
    }

    public void stopElevatorMotor() {
        leftElevatorMotor.set(0);
    }

    public void setBasePosition(double height) {
        leftElevatorMotor.setPosition(height);
    }

    public boolean isAtBottom() {
        return !bottomHallEffect.get();
    }

    public void resetElevatorEncoders() {
        leftElevatorMotor.setPosition(0);
        rightElevatorMotor.setPosition(0); // don't need to do this but a nice verification
    }

    /* SHOOTER MOUNT STATES */

    public void setShooterPosState(ShooterMountState newState) {
        shooterPosState = newState;
    }

    public ShooterMountState getShooterMountState() {
        return shooterPosState;
    }

    public String getShooterMountStateAsString() {
        return shooterPosState.toString();
    }

    // wpk move this to an appropriate area in the file.
    public double getSpeakerShooterAngle() {

        double result = 0.0;

        // deltaY is the difference in height of target (a little over the bottom of speaker opening) and the current
        // elevator position.
        double height = ShooterMountConstants.MidSpeakerHeight - Constants.ShooterMountConstants.SpeakerHeight ;
        height = Units.inchesToMeters(height ) ;

        var target = visionSub.getSpeakerTarget();

        double distance = 0.0 ;
        if (target.isPresent()) {
            // deltaX is the horizontal distance to the april tag (and the speaker)
            double x = target.get().getBestCameraToTarget().getX();
            double y = target.get().getBestCameraToTarget().getY();
            distance = Math.sqrt(x * x + y * y);
        } else {
            var speakerPose = visionSub.getSpeakerPose() ;
            if (speakerPose.isPresent()) {
                var speakerTranslation = speakerPose.get().toPose2d().getTranslation() ;
                distance = driveSub.getPose().getTranslation().getDistance(speakerTranslation) ;
            } else {
                return Constants.ShooterMountConstants.SpeakerAngle ;
            }
        }

        // Compute the angle and return it.
        result = Math.toDegrees(Math.atan2(height, distance)) ; // Make sure X,Y units match

        if (result > ShooterMountConstants.MaxAngleDegrees) result = ShooterMountConstants.MaxAngleDegrees;
         
        return (result);
    }





    /* TOLERANCES */

    private boolean isWithinAngleTolerance(double desiredAngle) {

        boolean withinTolerence = (getAngleCANCoderDegrees() >= desiredAngle - ShooterMountConstants.AngleTolerance) &&
                    (getAngleCANCoderDegrees() <= desiredAngle + ShooterMountConstants.AngleTolerance) ;

        return withinTolerence ;
    }

    private boolean isWithinAngleTolerance2() {
        boolean withinTolerence = (getAngleCANCoderDegrees() >= desiredDegrees - ShooterMountConstants.AngleTolerance) &&
                    (getAngleCANCoderDegrees() <= desiredDegrees + ShooterMountConstants.AngleTolerance) ;
        return withinTolerence ;
    }

    private boolean isWithinHeightTolerance(double desiredHeight) {
        return (getHeightInches() >= desiredHeight - ShooterMountConstants.HeightTolerance) &&
                (getHeightInches() <= desiredHeight + ShooterMountConstants.HeightTolerance);
    }

    public boolean isWithinPositionTolerance(double desiredAngle, double desiredHeight) {
        return isWithinAngleTolerance(desiredAngle) && isWithinHeightTolerance(desiredHeight);
    }

    private void logData() {
        Logger.recordOutput("ShooterMount/ShooterMountState", getShooterMountStateAsString());
        Logger.recordOutput("ShooterMount/Angle/CANCoderDegrees", getAngleCANCoderDegrees());
        Logger.recordOutput("ShooterMount/Angle/CANCoderRotations", angleCANCoder.getAbsolutePosition().getValue());
        Logger.recordOutput("ShooterMount/Angle/Talon", getTalonEncoderDegrees());
        Logger.recordOutput("ShooterMount/VoltageActual/AngleMotor", angleMotor.getMotorVoltage().getValue());
        Logger.recordOutput("ShooterMount/VoltageActual/ElevatorLeftMotor",
                leftElevatorMotor.getMotorVoltage().getValue());
        Logger.recordOutput("ShooterMount/VoltageActual/ElevatorRightMotor",
                rightElevatorMotor.getMotorVoltage().getValue());
        Logger.recordOutput("ShooterMount/ClosedLoopError/AngleMotor", angleMotor.getClosedLoopError().getValue());
        Logger.recordOutput("ShooterMount/ClosedLoopError/ElevatorLeftMotor",
                leftElevatorMotor.getClosedLoopError().getValue());
        Logger.recordOutput("ShooterMount/ClosedLoopError/ElevatorRightMotor",
                rightElevatorMotor.getClosedLoopError().getValue());
        Logger.recordOutput("ShooterMount/TempC/LeftElevatorMotor", leftElevatorMotor.getDeviceTemp().getValue());
        Logger.recordOutput("ShooterMount/TempC/RightElevatorMotor", rightElevatorMotor.getDeviceTemp().getValue());
        Logger.recordOutput("ShooterMount/ActualHeight", getHeightInches());
        Logger.recordOutput("ShooterMount/IsAtBottom", isAtBottom());
        Logger.recordOutput("ShooterMount/CurrentSupply/ElevatorLeft", leftElevatorMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("ShooterMount/CurrentSupply/ElevatorRight",
                rightElevatorMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("ShooterMount/CurrentSupply/Angle", angleMotor.getSupplyCurrent().getValue());
        Logger.recordOutput("ShooterMount/ControlMode/ElevatorLeft",
                leftElevatorMotor.getControlMode().getValue());
        Logger.recordOutput("ShooterMount/Height/RawElevatorLeftMotorRotations",
                leftElevatorMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("ShooterMount/Height/RawElevatorRightMotorRotations",
                rightElevatorMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("ShooterMount/Height/SetpointElevatorLeftRotations",
                leftElevatorMotor.getClosedLoopReference().getValue());
        Logger.recordOutput("ShooterMount/Height/SetpointElevatorRightRotations",
                rightElevatorMotor.getClosedLoopReference().getValue());
    }

    /* CONFIG */

    private void applyAngleMotorConfigs(InvertedValue inversion) {
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.Slot0.kP = ShooterMountConstants.AngleKP;
        talonConfigs.Slot0.kI = ShooterMountConstants.AngleKI;
        talonConfigs.Slot0.kD = ShooterMountConstants.AngleKD;
        talonConfigs.Slot0.kV = ShooterMountConstants.AngleFF;
        talonConfigs.Slot0.kG = ShooterMountConstants.AngleKG;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = talonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterMountConstants.AngleMMCruiseDegPerSec;
        motionMagicConfigs.MotionMagicAcceleration = ShooterMountConstants.AngleMMAcceleration;
        motionMagicConfigs.MotionMagicJerk = ShooterMountConstants.AngleMMJerk;

        talonConfigs.Feedback.FeedbackRemoteSensorID = angleCANCoder.getDeviceID();
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        applyMotorConfigs(angleMotor, "angleMotor", talonConfigs, inversion);
    }

    private void applyElevatorMotorConfigs(TalonFX motor, String motorName, InvertedValue inversion) {
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.Slot0.kP = ShooterMountConstants.ElevatorKP;
        talonConfigs.Slot0.kI = ShooterMountConstants.ElevatorKI;
        talonConfigs.Slot0.kD = ShooterMountConstants.ElevatorKD;
        talonConfigs.Slot0.kV = ShooterMountConstants.ElevatorFF;
        talonConfigs.Slot0.kG = ShooterMountConstants.ElevatorKG;
        talonConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        var motionMagicConfigs = talonConfigs.MotionMagic;

        double rotationsPerSecond = ShooterMountConstants.ElevatorMMCruiseInchesPerSec
                * ShooterMountConstants.RotationsPerElevatorInch;
        motionMagicConfigs.MotionMagicCruiseVelocity = rotationsPerSecond;

        double rotationsPerSecondPerSecond = (ShooterMountConstants.ElevatorMMInchesPerSecPerSec
                * ShooterMountConstants.RotationsPerElevatorInch) / 0.25;
        motionMagicConfigs.MotionMagicAcceleration = rotationsPerSecondPerSecond;

        // motionMagicConfigs.MotionMagicJerk = ShooterMountConstants.ElevatorMMJerk;
        motionMagicConfigs.MotionMagicJerk = rotationsPerSecondPerSecond / 0.1;

        applyMotorConfigs(motor, motorName, talonConfigs, inversion);
    }

    private void applyMotorConfigs(TalonFX motor, String motorName, TalonFXConfiguration configs,
            InvertedValue inversion) {

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        /* APPLY PID CONFIGS */

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(configs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply talon configs to " + motorName + " error code: " + status.toString());
        }

        /* SET & APPLY INVERSION CONFIGS */

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

        motorOutputConfigs.Inverted = inversion;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(motorOutputConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply motor output configs to " + motor + " error code: " + status.toString());
        }

        /* SET & APPLY CURRENT LIMIT CONFIGS */

        CurrentLimitsConfigs currentLimitConfigs = configs.CurrentLimits;
        currentLimitConfigs.SupplyCurrentLimit = ShooterMountConstants.SupplyCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimitEnable = true;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(currentLimitConfigs, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply current limit configs to " + motor + " error code: " + status.toString());
        }
    }

    private void applyAngleEncoderConfigs() {
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        var canCoderConfiguration = new CANcoderConfiguration();
        magnetConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        if (!Robot.isSimulation()) {
            magnetConfig.MagnetOffset = ShooterMountConstants.AngleCANCoderMagnetOffset;
        } else {
            magnetConfig.MagnetOffset = 0.0;
        }

        magnetConfig.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfiguration.MagnetSensor = magnetConfig;

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = angleCANCoder.getConfigurator().apply(canCoderConfiguration, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply CANCoder configs to angle encoder, error code: " + status.toString());
        }

        for (int i = 0; i < 5; ++i) {
            status = angleCANCoder.getConfigurator().apply(magnetConfig, 0.05);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println(
                    "Could not apply magnet configs to angle encoder, error code: " + status.toString());
        }
    }

    private void setNeutralMode(NeutralModeValue angleMotorMode, NeutralModeValue elevatorMotorMode) {
        angleMotor.setNeutralMode(angleMotorMode);
        leftElevatorMotor.setNeutralMode(elevatorMotorMode);
        rightElevatorMotor.setNeutralMode(elevatorMotorMode);
    }

    /* SIMULATION */

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(angleMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(leftElevatorMotor, 0.001);
        PhysicsSim.getInstance().addTalonFX(rightElevatorMotor, 0.001);
        bottomHallEffectSim = new DIOSim(bottomHallEffect);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }

        angleMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double angleVoltage = angleMotorSim.getMotorVoltage();
        angleMotorModel.setInputVoltage(angleVoltage);
        angleMotorModel.update(0.02);
        angleMotorSim.setRotorVelocity(angleMotorModel.getAngularVelocityRPM() / 60.0);
        angleMotorSim.setRawRotorPosition(angleMotorModel.getAngularPositionRotations());

        leftElevatorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double leftVoltage = leftElevatorMotorSim.getMotorVoltage();
        leftElevatorMotorModel.setInputVoltage(leftVoltage);
        leftElevatorMotorModel.update(0.02);
        leftElevatorMotorSim.setRotorVelocity(leftElevatorMotorModel.getAngularVelocityRPM() / 60.0);
        leftElevatorMotorSim.setRawRotorPosition(leftElevatorMotorModel.getAngularPositionRotations());

        rightElevatorMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double rightVoltage = rightElevatorMotorSim.getMotorVoltage();
        rightElevatorMotorModel.setInputVoltage(rightVoltage);
        rightElevatorMotorModel.update(0.02);
        rightElevatorMotorSim.setRotorVelocity(rightElevatorMotorModel.getAngularVelocityRPM() / 60.0);
        rightElevatorMotorSim.setRawRotorPosition(rightElevatorMotorModel.getAngularPositionRotations());

        bottomHallEffectSim.setValue(isWithinHeightTolerance(0));
    }
}