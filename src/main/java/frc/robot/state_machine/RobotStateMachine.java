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

  private FiniteStateMachine stateMachine;
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
    if (robotStateMachine == null) {
      robotStateMachine = new RobotStateMachine();
    }

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
            .sourceState(intaking)
            .eventType(PieceDetectedMidHigh.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(indexerCarryMidHighCommand);
                })
            .targetState(carrying)
            .build();

    Transition transitionC =
        new TransitionBuilder()
            .name("transitionC")
            .sourceState(carrying)
            .eventType(ScorePressed.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(stageCommand);
                })
            .targetState(staged)
            .build();

    Transition transitionD =
        new TransitionBuilder()
            .name("transitionD")
            .sourceState(staged)
            .eventType(FinishScorePressed.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(scoreCommand);
                })
            .targetState(readyToIntake)
            .build();

    Transition transitionF =
        new TransitionBuilder()
            .name("transitionF")
            .sourceState(intaking)
            .eventType(PieceDetectedLow.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(indexerCarryLowCommand);
                })
            .targetState(holdingLow)
            .build();

    Transition transitionG =
        new TransitionBuilder()
            .name("transitionG")
            .sourceState(holdingLow)
            .eventType(PlaceButtonPressed.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(indexerPlaceCommand);
                })
            .targetState(readyToIntake)
            .build();

    Transition transitionH =
        new TransitionBuilder()
            .name("transitionH")
            .sourceState(holdingLow)
            .eventType(SwitchToLow.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(indexerSwitchToMidHighCommand);
                })
            .targetState(carrying)
            .build();

    Transition transitionI =
        new TransitionBuilder()
            .name("transitionI")
            .sourceState(carrying)
            .eventType(SwitchToMidHigh.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(indexerSwitchToLowCommand);
                })
            .targetState(holdingLow)
            .build();

    Transition transitionJ =
        new TransitionBuilder()
            .name("transitionJ")
            .sourceState(staged)
            .eventType(IntakePressed.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(smartIntakeCommand);
                })
            .targetState(intaking)
            .build();

    Transition transitionK =
        new TransitionBuilder()
            .name("transitionK")
            .sourceState(staged)
            .eventType(SwitchToLow.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(stagedToLowCommand);
                })
            .targetState(readyToIntake)
            .build();

    Transition transitionL =
        new TransitionBuilder()
            .name("transitionL")
            .sourceState(intaking)
            .eventType(IntakeReleased.class)
            .eventHandler((event) -> {})
            .targetState(readyToIntake)
            .build();

    stateMachine =
        new FiniteStateMachineBuilder(states, readyToIntake)
            .registerTransition(transitionA)
            .registerTransition(transitionB)
            .registerTransition(transitionC)
            .registerTransition(transitionD)
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
      eventName = stateMachine.fire(event).getName();
    } catch (FiniteStateMachineException e) {
      eventName = stateMachine.getCurrentState().getName();
    }
    return eventName;
  }

  public String getCurrentState() {
    return stateMachine.getCurrentState().getName();
  }

  public String getLastEvent() {
    return stateMachine.getLastEvent().getName();
  }

  public String getLastTransition() {
    return stateMachine.getLastTransition().getName();
  }
}