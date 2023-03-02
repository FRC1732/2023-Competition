package frc.robot.state_machine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.TransitionCommands.*;
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
  private RobotContainer robotContainer;

  private FiniteStateMachine stateMachine;
  private SmartIntakeCommand smartIntakeCommand;
  private ResetToReadyCommand resetToReadyCommand;
  private Command carryGamePieceCommand;
  private StageGamePieceCommand stageGamePieceCommand;
  private Command scoreWithHolderCommand;
  private HoldGamePieceLowCommand holdGamePieceLowCommand;
  private ScoreLowCommand scoreLowCommand;
  private Command switchFromCarryingToHoldingLowCommand;
  private Command unstageGamePieceCommand;

  public RobotStateMachine(RobotContainer container) {
    robotContainer = container;

    smartIntakeCommand = new SmartIntakeCommand(robotContainer, this);

    stageGamePieceCommand = new StageGamePieceCommand(robotContainer);

    holdGamePieceLowCommand = new HoldGamePieceLowCommand(robotContainer);
    scoreLowCommand = new ScoreLowCommand(robotContainer);
    resetToReadyCommand = new ResetToReadyCommand(robotContainer);
    scoreWithHolderCommand =
        Commands.sequence(
            new DeployExtenderCommand(robotContainer),
            new OpenHolderCommand(robotContainer),
            new RetractExtenderCommand(robotContainer),
            new ResetToReadyCommand(robotContainer));
    carryGamePieceCommand =
        Commands.sequence(
            new MoveIndexerToScoringCommand(robotContainer),
            new LowerElevatorToTransferCommand(robotContainer));
    switchFromCarryingToHoldingLowCommand =
        Commands.sequence(
            new OpenHolderCommand(robotContainer),
            new MoveElevatorToNeutralCommand(robotContainer),
            new HoldGamePieceLowCommand(robotContainer));
    unstageGamePieceCommand =
        Commands.sequence(
            new UnstageGamePieceCommand(robotContainer), new ResetToReadyCommand(robotContainer));

    State readyToIntake = new State("readyToIntake");
    State intaking = new State("intaking");
    State holdingLow = new State("holdingLow");
    State carrying = new State("carrying");
    State staged = new State("staged");

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
                  CommandScheduler.getInstance().schedule(carryGamePieceCommand);
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
                  CommandScheduler.getInstance().schedule(stageGamePieceCommand);
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
                  CommandScheduler.getInstance().schedule(scoreWithHolderCommand);
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
                  CommandScheduler.getInstance().schedule(holdGamePieceLowCommand);
                })
            .targetState(holdingLow)
            .build();

    Transition transitionG =
        new TransitionBuilder()
            .name("transitionG")
            .sourceState(holdingLow)
            .eventType(ScorePressed.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(scoreLowCommand);
                })
            .targetState(readyToIntake)
            .build();

    Transition transitionH =
        new TransitionBuilder()
            .name("transitionH")
            .sourceState(holdingLow)
            .eventType(SwitchToMidHigh.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(carryGamePieceCommand);
                })
            .targetState(carrying)
            .build();

    Transition transitionH2 =
        new TransitionBuilder()
            .name("transitionH2")
            .sourceState(carrying)
            .eventType(SwitchToLow.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(switchFromCarryingToHoldingLowCommand);
                })
            .targetState(holdingLow)
            .build();

    // Transition transitionI =
    //     new TransitionBuilder()
    //         .name("transitionI")
    //         .sourceState(staged)
    //         .eventType(IntakePressed.class)
    //         .eventHandler(
    //             (event) -> {
    //               CommandScheduler.getInstance().schedule(smartIntakeCommand);
    //             })
    //         .targetState(intaking)
    //         .build();

    Transition transitionJ =
        new TransitionBuilder()
            .name("transitionJ")
            .sourceState(staged)
            .eventType(SwitchToLow.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(unstageGamePieceCommand);
                })
            .targetState(readyToIntake)
            .build();

    Transition transitionK =
        new TransitionBuilder()
            .name("transitionK")
            .sourceState(intaking)
            .eventType(IntakeReleased.class)
            .eventHandler(
                (event) -> {
                  CommandScheduler.getInstance().schedule(resetToReadyCommand);
                })
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
            .registerTransition(transitionH2)
            // .registerTransition(transitionI)
            .registerTransition(transitionJ)
            .registerTransition(transitionK)
            .build();
  }

  public String fireEvent(org.jeasy.states.api.Event event) {
    String startingState = getCurrentState();
    String eventName;
    try {
      eventName = stateMachine.fire(event).getName();
    } catch (FiniteStateMachineException e) {
      System.out.println("State Machine Exception!!!!!!!!!!");
      e.printStackTrace();
      eventName = getCurrentState();
    }
    System.out.println(
        "Start: " + startingState + "Event: " + event.getName() + "End: " + getCurrentState());
    return eventName;
  }

  public String getCurrentState() {
    try {
      return stateMachine.getCurrentState().getName();
    } catch (Exception e) {
      return "Unknown State";
    }
  }

  public String getLastEvent() {
    try {
      return stateMachine.getLastEvent().getName();
    } catch (Exception e) {
      return "Unknown Last Event";
    }
  }

  public String getLastTransition() {
    try {
      return stateMachine.getLastTransition().getName();
    } catch (Exception e) {
      return "Unknown Transition";
    }
  }
}
