package frc.robot.state_machine;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SmartIntakeCommand;
import frc.robot.state_machine.events.*;
import java.util.HashSet;
import java.util.Set;
import org.jeasy.states.api.FiniteStateMachine;
import org.jeasy.states.api.FiniteStateMachineException;
import org.jeasy.states.api.State;
import org.jeasy.states.api.Transition;
import org.jeasy.states.core.FiniteStateMachineBuilder;
import org.jeasy.states.core.TransitionBuilder;
import org.littletonrobotics.junction.Logger;

public class RobotStateMachine{
  // RobotContainer singleton
  private static RobotStateMachine robotStateMachine = new RobotStateMachine();

  private FiniteStateMachine _stateMachine;
  private SmartIntakeCommand smartIntakeCommand = new SmartIntakeCommand();

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotStateMachine getInstance() {
    return robotStateMachine;
  }

  private RobotStateMachine() {
    State readyToIntake = new State("readyToIntake");
    State intaking = new State("intaking");
    State holdingLow = new State("holdingLow");
    State carrying = new State("carrying");
    State staged = new State("staged");
    State scoring = new State("scoring");

    Set<State> states = new HashSet<>();
    states.add(readyToIntake);
    states.add(intaking);
    states.add(holdingLow);
    states.add(carrying);
    states.add(staged);
    states.add(scoring);

    Transition transitionA = new TransitionBuilder()
        .name("transitionA")
        .sourceState(readyToIntake) // if we are in state readyToIntake
        .eventType(IntakePressed.class) // and the event IntakePressed occurs
        .eventHandler(
            (event) -> {
              CommandScheduler.getInstance().schedule(smartIntakeCommand);
              Logger
                  .getInstance()
                  .recordOutput("StateMachine/CurrentState", intaking.getName());
            }) // we should perform the action
        .targetState(intaking) // and make a transition to the state unlocked
        .build();

        Transition transitionB = 
        new TransitionBuilder()
            .name("transitionB")
            .sourceState(intaking) // if we are in state intaking
            .eventType(PieceDetectedMidHigh.class) // and the event IntakePressed occurs
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(smartIntakeCommand);
                  Logger
                      .getInstance()
                      .recordOutput("StateMachine/CurrentState", carrying.getName());
                }) // we should perform the action
            .targetState(carrying) // and make a transition to the state carrying
            .build();

    _stateMachine = new FiniteStateMachineBuilder(states, readyToIntake)
        .registerTransition(transitionA)
        .build();
  }

  public String fireEvent(org.jeasy.states.api.Event event) {
    String eventName;
    try {
      eventName = _stateMachine.fire(event).getName();
    } catch (FiniteStateMachineException e) {
      eventName = _stateMachine.getCurrentState().getName();
    }
    return eventName;
  }
}
