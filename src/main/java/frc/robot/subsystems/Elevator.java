// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  /** Creates a new Elevator. */
  private SparkMax elevator; // Only one NEO
  SparkMaxConfig config;
  DigitalInput upperLimitSwitch;
  DigitalInput lowerLimitSwitch;
  SparkAbsoluteEncoder encoder;
  boolean top;
  boolean bottom;

  public Elevator(int elevatorID1, int upperLimitSwitchID, int lowerLimitSwitchID) {
    elevator = new SparkMax(elevatorID1, MotorType.kBrushless);
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
    if ((upperLimitSwitch.get()&&encoder.getVelocity()>0) || (lowerLimitSwitch.get()&&encoder.getVelocity()<0)) {// TODO: Recheck limit switch code
      elevator.stopMotor();
      return;
    }
    elevator.setVoltage(voltage);
  }

  public void setSpeed(double speed) {
    if ((upperLimitSwitch.get()&&encoder.getVelocity()>0) || (lowerLimitSwitch.get()&&encoder.getVelocity()<0)) {// TODO: Recheck limit switch code
      elevator.stopMotor();
      return;
    }
    elevator.set(speed);
  }

  public void toSetpoint(double setpoint) {
    elevator.getClosedLoopController().setSetpoint(setpoint, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    top = upperLimitSwitch.get();
    bottom = lowerLimitSwitch.get();

    if(top || bottom){
      elevator.stopMotor();
    }
  }
}
