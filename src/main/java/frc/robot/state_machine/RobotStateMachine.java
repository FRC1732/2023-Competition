package frc.robot.state_machine;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.IndexerCommands.IndexerCarryLowCommand;
import frc.robot.commands.IndexerCommands.IndexerCarryMidHighCommand;
import frc.robot.commands.IndexerCommands.IndexerPlaceCommand;
import frc.robot.commands.IndexerCommands.IndexerSwitchToLowCommand;
import frc.robot.commands.IndexerCommands.IndexerSwitchToMidHighCommand;
import frc.robot.commands.ResetFromScoringCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.SmartIntakeCommand;
import frc.robot.commands.StageCommand;
import frc.robot.commands.StagedToLowCommand;
import frc.robot.state_machine.events.*;
import java.util.HashSet;
import java.util.Set;
import org.jeasy.states.api.FiniteStateMachine;
import org.jeasy.states.api.FiniteStateMachineException;
import org.jeasy.states.api.State;
import org.jeasy.states.api.Transition;
import org.jeasy.states.core.FiniteStateMachineBuilder;
import org.jeasy.states.core.TransitionBuilder;

public class RobotStateMachine {
  // RobotContainer singleton
  private static RobotStateMachine robotStateMachine = new RobotStateMachine();

  private FiniteStateMachine _stateMachine;
  private SmartIntakeCommand smartIntakeCommand = new SmartIntakeCommand();
  private IndexerCarryMidHighCommand indexerCarryMidHighCommand = new IndexerCarryMidHighCommand();
  private StageCommand stageCommand = new StageCommand();
  private ScoreCommand scoreCommand = new ScoreCommand();
  private ResetFromScoringCommand resetFromScoringCommand = new ResetFromScoringCommand();
  private IndexerCarryLowCommand indexerCarryLowCommand = new IndexerCarryLowCommand();
  private IndexerPlaceCommand indexerPlaceCommand = new IndexerPlaceCommand();
  private IndexerSwitchToMidHighCommand indexerSwitchToMidHighCommand =
      new IndexerSwitchToMidHighCommand();
  private IndexerSwitchToLowCommand indexerSwitchToLowCommand = new IndexerSwitchToLowCommand();
  private StagedToLowCommand stagedToLowCommand = new StagedToLowCommand();

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

    Transition transitionA =
        new TransitionBuilder()
            .name("transitionA")
            .sourceState(readyToIntake) // if we are in state readyToIntake
            .eventType(IntakePressed.class) // and the event IntakePressed occurs
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(smartIntakeCommand);
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
                  CommandScheduler.getInstance().schedule(indexerCarryMidHighCommand);
                }) // we should perform the action
            .targetState(carrying) // and make a transition to the state carrying
            .build();

    Transition transitionC =
        new TransitionBuilder()
            .name("transitionC")
            .sourceState(carrying) // if we are in state intaking
            .eventType(ScorePressed.class) // and the event IntakePressed occurs
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(stageCommand);
                }) // we should perform the action
            .targetState(staged) // and make a transition to the state carrying
            .build();

    Transition transitionD =
        new TransitionBuilder()
            .name("transitionD")
            .sourceState(staged) // if we are in state intaking
            .eventType(FinishScorePressed.class) // and the event IntakePressed occurs
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(scoreCommand);
                }) // we should perform the action
            .targetState(scoring) // and make a transition to the state carrying
            .build();

    Transition transitionE =
        new TransitionBuilder()
            .name("transitionE")
            .sourceState(staged) // if we are in state intaking
            .eventType(ScoringFinishes.class) // and the event IntakePressed occurs
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(resetFromScoringCommand);
                }) // we should perform the action
            .targetState(readyToIntake) // and make a transition to the state carrying
            .build();

    Transition transitionF =
        new TransitionBuilder()
            .name("transitionF")
            .sourceState(intaking) // if we are in state intaking
            .eventType(PieceDetectedLow.class) // and the event IntakePressed occurs
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(indexerCarryLowCommand);
                }) // we should perform the action
            .targetState(holdingLow) // and make a transition to the state carrying
            .build();

    Transition transitionG =
        new TransitionBuilder()
            .name("transitionG")
            .sourceState(holdingLow) // if we are in state intaking
            .eventType(PlaceButtonPressed.class) // and the event IntakePressed occurs
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(indexerPlaceCommand);
                }) // we should perform the action
            .targetState(readyToIntake) // and make a transition to the state carrying
            .build();

    Transition transitionH =
        new TransitionBuilder()
            .name("transitionH")
            .sourceState(holdingLow) // if we are in state intaking
            .eventType(SwitchToLow.class) // and the event IntakePressed occurs
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(indexerSwitchToMidHighCommand);
                }) // we should perform the action
            .targetState(carrying) // and make a transition to the state carrying
            .build();

    Transition transitionI =
        new TransitionBuilder()
            .name("transitionI")
            .sourceState(carrying) // if we are in state intaking
            .eventType(SwitchToMidHigh.class) // and the event IntakePressed occurs
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(indexerSwitchToLowCommand);
                }) // we should perform the action
            .targetState(holdingLow) // and make a transition to the state carrying
            .build();

    Transition transitionJ =
        new TransitionBuilder()
            .name("transitionJ")
            .sourceState(staged) // if we are in state intaking
            .eventType(IntakePressed.class) // and the event IntakePressed occurs
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(smartIntakeCommand);
                }) // we should perform the action
            .targetState(intaking) // and make a transition to the state carrying
            .build();

    Transition transitionK =
        new TransitionBuilder()
            .name("transitionK")
            .sourceState(staged) // if we are in state intaking
            .eventType(SwitchToLow.class) // and the event IntakePressed occurs
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(stagedToLowCommand);
                }) // we should perform the action
            .targetState(readyToIntake) // and make a transition to the state carrying
            .build();

    Transition transitionL =
        new TransitionBuilder()
            .name("transitionL")
            .sourceState(intaking) // if we are in state intaking
            .eventType(IntakeReleased.class) // and the event IntakePressed occurs
            .eventHandler((event) -> {}) // we should perform the action
            .targetState(readyToIntake) // and make a transition to the state carrying
            .build();

    _stateMachine =
        new FiniteStateMachineBuilder(states, readyToIntake)
            .registerTransition(transitionA)
            .registerTransition(transitionB)
            .registerTransition(transitionC)
            .registerTransition(transitionD)
            .registerTransition(transitionE)
            .registerTransition(transitionF)
            .registerTransition(transitionG)
            .registerTransition(transitionH)
            .registerTransition(transitionI)
            .registerTransition(transitionJ)
            .registerTransition(transitionK)
            .registerTransition(transitionL)
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

  public String getCurrentState() {
    return _stateMachine.getCurrentState().getName();
  }

  public String getLastEvent() {
    return _stateMachine.getLastEvent().getName();
  }

  public String getLastTransition() {
    return _stateMachine.getLastTransition().getName();
  }
}
