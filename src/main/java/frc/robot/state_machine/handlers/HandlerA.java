package frc.robot.state_machine.handlers;

import frc.robot.state_machine.events.IntakePressed;
import org.jeasy.states.api.EventHandler;

// ReadyToIntake->Intaking
public class HandlerA implements EventHandler<IntakePressed> {
  public void handleEvent(IntakePressed event) {
    // indexer go to intake position
    // indexer intake
  }
}
