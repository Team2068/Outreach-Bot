package frc.robot.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import frc.robot.swerve.Swerve;


public class Module {
    public final TalonFX drive;
    public final SparkMax steer;
    public final Swerve.Encoder encoder;
    SparkMaxConfig steerConfig;

    double desiredAngle;
    PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double STEER_REDUCTION = 26.09;

    public Module(ShuffleboardLayout tab, int driveID, int steerID, int encoderID, boolean heliumEncoder) {
        drive = new TalonFX(driveID, "Swerve");
        steer = new SparkMax(steerID, MotorType.kBrushless);
        encoder = (heliumEncoder) ? new Swerve.Canand(encoderID) : new Swerve.Cancoder(encoderID);
        
        steerConfig = new SparkMaxConfig();

        steerConfig.smartCurrentLimit(80);

        steerConfig.inverted(true);
        steerConfig.idleMode(IdleMode.kBrake);
        

        steerConfig.closedLoop.pid(0.2, 0, 0);
        steerConfig.closedLoop.positionWrappingEnabled(true);
        steerConfig.closedLoop.positionWrappingMaxInput(Swerve.PI2);

        steerConfig.signals.primaryEncoderPositionAlwaysOn(false);
        steerConfig.signals.primaryEncoderPositionPeriodMs(20);



        steerConfig.encoder.positionConversionFactor(Math.PI * STEER_REDUCTION);
        steerConfig.encoder.velocityConversionFactor(Math.PI * STEER_REDUCTION / 60);
 
        steer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        steer.getEncoder().setPosition(angle());

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.CurrentLimits.StatorCurrentLimit = 80;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        driveConfig.CurrentLimits.SupplyCurrentLimit = 20;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        drive.getConfigurator().apply(driveConfig);

        tab.addDouble("Absolute Module Angle", () -> Math.toDegrees(angle()));
        tab.addDouble("Current Angle", () ->Math.toDegrees(steer.getEncoder().getPosition()));
        tab.addDouble("Angle Difference", () -> Math.toDegrees(angle() - (steer.getEncoder().getPosition() * Swerve.PI2))); //TODO: Math.toDegrees(angle()) - steer.getPosition().getValueAsDouble();
        tab.addDouble("Target Angle", () -> Math.toDegrees(desiredAngle));
        tab.addBoolean("Active", encoder::connected);
    }

    public void resetDrivePosition() {
        drive.setPosition(0.0);
    }

    public void syncEncoders() {
        steer.getEncoder().setPosition(angle());
    }

    public void zeroAbsolute() {
        encoder.zero();
    }

    public double drivePosition() {
        return drive.getPosition().getValueAsDouble() * .500 * WHEEL_DIAMETER;
    }

    public LinearVelocity velocity() {
        return MetersPerSecond.of(drive.getVelocity().getValueAsDouble() * Swerve.PI2 * .500 * WHEEL_DIAMETER); //TODO: return MetersPerSecond.of(drive.getVelocity().getValueAsDouble() * Math.PI * WHEEL_DIAMETER);
        
    }

    public Voltage voltage(){
        return drive.getMotorVoltage().getValue();
    }

    public double angle() {
        return encoder.angle();
    }

    public AngularVelocity steerVelocity(){
        return encoder.velocity();
    }

    public Voltage steerVoltage(){
        return Volts.of(steer.getBusVoltage());
    }

    public void stop() {
        drive.stopMotor();
        steer.stopMotor();
    }

    public void set(double driveVolts, double targetAngle) {
        double normalized = MathUtil.inputModulus(targetAngle, 0, Swerve.PI2);
        // syncEncoders();
        drive.set(driveVolts);
        steer.getClosedLoopController().setSetpoint(normalized, ControlType.kPosition);
    }

    public void setSteer(double steerVolts){
        // syncEncoders();
        drive.set(0);
        steer.set(steerVolts);
    }
}