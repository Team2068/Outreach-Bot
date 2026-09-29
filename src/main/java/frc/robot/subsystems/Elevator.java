// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final SparkMax elevator1;
  SparkMaxConfig config1;
  private final SparkMax elevator2;
  SparkMaxConfig config2;

  public Elevator(int elevatorID1, int elevatorID2) {
      elevator1 = new SparkMax(elevatorID1,MotorType.kBrushless);
      elevator2 = new SparkMax(elevatorID2, MotorType.kBrushless);

      config1
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);

      config2
        .apply(config1)
        .follow(elevatorID1)
        .inverted(true);

  }

  public void setVoltage(int voltage){}
  public void setSpeed(int speed){}
  public void toSetpoint(int setpoint){}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
