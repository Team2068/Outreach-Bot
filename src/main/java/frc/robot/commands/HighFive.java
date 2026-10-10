package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.utility.IO;

public class HighFive extends Command {
  Elevator elevator;
  int highfiveHeight = 1;

  public HighFive(IO io) {
    this.elevator = io.elevator;
    addRequirements(io.elevator);
  }

  @Override
  public void initialize() {
    elevator.setPosition(highfiveHeight); // figure out positions
    new WaitCommand(2.3);
    elevator.setPosition(0); // back to bottom
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return(elevator.getPosition() >= highfiveHeight);
  }
}
