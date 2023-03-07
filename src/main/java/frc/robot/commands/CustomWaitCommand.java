package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CustomWaitCommand extends WaitCommand {

  public CustomWaitCommand(double seconds) {
    super(seconds);
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }
}
