package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private SparkMax elevator; 
  SparkMaxConfig config;
  DigitalInput upperLimitSwitch;
  DigitalInput lowerLimitSwitch;
  SparkAbsoluteEncoder encoder;
  boolean top;
  boolean bottom;

  private double maxHeight = 1; // TODO GET MAX HEIGHT
  private double minHeight = 0;

  public Elevator(int elevatorID, int upperLimitSwitchID, int lowerLimitSwitchID) {
    elevator = new SparkMax(elevatorID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    encoder = elevator.getAbsoluteEncoder();
    upperLimitSwitch = new DigitalInput(upperLimitSwitchID);
    lowerLimitSwitch = new DigitalInput(lowerLimitSwitchID);
    config// TODO: Check config values
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50)
        .closedLoopRampRate(60)
        .closedLoop
          .pid(0, 0, 0);

    elevator.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stop() {
    elevator.stopMotor();
  }

  public void setVoltage(double voltage) {
    elevator.setVoltage(voltage);
  }

  public void setSpeed(double speed) {
    elevator.set(speed);
  }

  public void setPosition(double position) {
    elevator.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
  }

  public double getPosition(){
    return elevator.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    top = upperLimitSwitch.get();
    bottom = lowerLimitSwitch.get();

    if(top){
      elevator.getEncoder().setPosition(maxHeight);
      stop();
    }
    else if(bottom){
      elevator.getEncoder().setPosition(minHeight);
      stop();
    }
  }
}
