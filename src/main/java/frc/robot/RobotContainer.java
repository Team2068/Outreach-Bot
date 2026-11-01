// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.AutomatedController;
import frc.robot.commands.DefaultDrive;
import frc.robot.utility.IO;

public class RobotContainer {
  public IO io = new IO();
  public final AutomatedController main;

  final SendableChooser<Integer> driver_selector = new SendableChooser<Integer>();

  public RobotContainer() {
    main = new AutomatedController(0, io);
    
    io.chassis.setDefaultCommand(new DefaultDrive(io, main.controller));
    SmartDashboard.putData("Main Controller Mode", main.selector);
    CanandEventLoop.getInstance();
  }

}
