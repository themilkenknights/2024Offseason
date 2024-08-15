package com.pathplanner.lib.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerCommandUtil extends Command {
  boolean triggerBoolean = false;

  @Override
  public void execute() {}

  @Override
  public void initialize() {

    super.initialize();
    triggerBoolean = true;
  }

  @Override
  public boolean isFinished() {
    // TODO: figure out how to tell if parrent command is done
    return super.isFinished();
  }

  public Trigger getTrigger() {
    return new Trigger(() -> triggerBoolean);
  }

  @Override
  public void end(boolean interrupted) {

    triggerBoolean = false;
  }
}
