// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InstrumentedSequentialCommandGroup extends Command {
  private final List<Command> commandList = new ArrayList<>();
  private int currentCommandIndex = -1;
  private boolean runWhenDisabled = true;
  private InterruptionBehavior interruptBehavior = InterruptionBehavior.kCancelIncoming;

  private final List<Consumer<Command>> m_initActions = new ArrayList<>();
  private final List<Consumer<Command>> m_finishActions = new ArrayList<>();

  /**
   * Creates a new SequentialCommandGroup. The given commands will be run sequentially,
   * with the composition finishing when the last command finishes.
   * @param commands the commands to include in this composition.
   */
  public InstrumentedSequentialCommandGroup(Command... commands) {
    addCommands(commands);
  }

  public final void addCommands(Command... commands) {
    if (currentCommandIndex != -1) {
      throw new IllegalStateException(
          "Commands cannot be added to a composition while it's running");
    }

    CommandScheduler.getInstance().registerComposedCommands(commands);

    for (Command command : commands) {
      commandList.add(command);
      m_requirements.addAll(command.getRequirements());
      runWhenDisabled &= command.runsWhenDisabled();
      if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        interruptBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }

  @Override
  public final void initialize() {
    currentCommandIndex = 0;

    if (!commandList.isEmpty()) {
      commandList.get(0).initialize();
    }
  }

  @Override
  public final void execute() {
    if (commandList.isEmpty()) {
      return;
    }

    Command currentCommand = commandList.get(currentCommandIndex);

    currentCommand.execute();
    if (currentCommand.isFinished()) {
      currentCommand.end(false);
      for (Consumer<Command> action : m_finishActions) {
        action.accept(currentCommand);
      }

      currentCommandIndex++;
      if (currentCommandIndex < commandList.size()) {
        commandList.get(currentCommandIndex).initialize();
        for (Consumer<Command> action : m_initActions) {
          action.accept(commandList.get(currentCommandIndex));
        }
      }
    }
  }

  @Override
  public final void end(boolean interrupted) {
    if (interrupted
        && !commandList.isEmpty()
        && currentCommandIndex > -1
        && currentCommandIndex < commandList.size()) {
      commandList.get(currentCommandIndex).end(true);
    }
    currentCommandIndex = -1;
  }

  @Override
  public final boolean isFinished() {
    return currentCommandIndex == commandList.size();
  }

  @Override
  public boolean runsWhenDisabled() {
    return runWhenDisabled;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return interruptBehavior;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addIntegerProperty("index", () -> currentCommandIndex, null);
  }

  public void onCommandInitialize(Consumer<Command> action) {
    m_initActions.add(requireNonNullParam(action, "action", "onCommandInitialize"));
  }

  public void onCommandFinish(Consumer<Command> action) {
    m_finishActions.add(requireNonNullParam(action, "action", "onCommandFinish"));
  }

}
