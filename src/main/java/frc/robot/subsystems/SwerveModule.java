package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {

    CANSparkMax turnMotor;
    TalonFX driveMotor;

    AbsoluteEncoder turnEncoder;

    SparkMaxPIDController turningPIDController;

    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int driveCANID, int turnCANID) {
        turnMotor = new CANSparkMax(turnCANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        driveMotor = new TalonFX(driveCANID);

        turnMotor.restoreFactoryDefaults();
        turnEncoder = turnMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        turningPIDController = turnMotor.getPIDController();
        turningPIDController.setFeedbackDevice(turnEncoder);
        turningPIDController.setP(SwerveModuleConstants.turnP);
        turningPIDController.setI(SwerveModuleConstants.turnI);
        turningPIDController.setD(SwerveModuleConstants.turnD);
        turningPIDController.setFF(SwerveModuleConstants.turnFF);

        turnEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor);
        turnEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor);
            
        turnEncoder.setInverted(true);
        turningPIDController.setPositionPIDWrappingEnabled(true);
        turningPIDController.setPositionPIDWrappingMinInput(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput);
        turningPIDController.setPositionPIDWrappingMaxInput(SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);

        turnMotor.setSmartCurrentLimit(SwerveModuleConstants.turningCurrentLimit);
        turnMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.burnFlash();

        var config = new TalonFXConfiguration();
        config.Slot0.kP = SwerveModuleConstants.driveP;
        config.Slot0.kI = SwerveModuleConstants.driveI;
        config.Slot0.kD = SwerveModuleConstants.driveD;
        config.Slot0.kV = SwerveModuleConstants.driveFF;


        config.Slot0.kS = SwerveModuleConstants.driveKs;
        config.CurrentLimits.StatorCurrentLimit = SwerveModuleConstants.driveCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = SwerveModuleConstants.kDriveMetersPerRev;
        driveMotor.getConfigurator().apply(config);






    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity().getValue(), new Rotation2d(turnEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValue(), new Rotation2d(turnEncoder.getPosition()));

    }

    public void setDesiredState(SwerveModuleState state) {

        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, new Rotation2d(turnEncoder.getPosition()));
        turningPIDController.setReference(optimizedState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
        var requestedVoltage = new VelocityVoltage(optimizedState.speedMetersPerSecond);
        driveMotor.setControl(requestedVoltage);

        m_desiredState = state;
    }

    public void resetEncoders() {
        driveMotor.setRotorPosition(0);
    }

    public static class SwerveModuleConstants {
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static int turningCurrentLimit = 30;
        public static double kTurningEncoderPositionPIDMinInput = 0;
        public static double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor;
        public static double turnP;
        public static double turnI = 0;
        public static double turnD = 0;
        public static double turnFF = 0;

        public static int driveCurrentLimit = 100;
        public static double driveP;
        public static double driveI;
        public static double driveD;
        public static double driveFF;
        public static double driveKs;
        public static double wheelDiameter = Units.inchesToMeters(4);
        public static double wheelCircumference = wheelDiameter * Math.PI;
        public static double driveRatio = 50./14.*17./27.*45./15.;    //MK4I L2
        public static double kDriveMetersPerRev = wheelCircumference/driveRatio;


    }


}
