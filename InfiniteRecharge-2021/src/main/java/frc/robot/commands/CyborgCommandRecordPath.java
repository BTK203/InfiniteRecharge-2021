// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.PathRecorder;
import frc.robot.util.PositionTracker;

public class CyborgCommandRecordPath extends CommandBase {
  private PathRecorder recorder;
  private PositionTracker tracker;

  /** Creates a new CyborgCommandRecordPath. */
  public CyborgCommandRecordPath(PositionTracker tracker) {
    // this.recorder = new PathRecorder("D:\\_Users\\Brach\\projects\\Test\\FRC\\out.txt"); //TODO: DELETE
    this.recorder = new PathRecorder("/home/lvuser/points.txt");
    this.tracker = tracker;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    recorder.openFile();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    recorder.recordPoint(tracker.getPositionAndHeading());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try {
      recorder.flushFile();
      recorder.closeFile();
      DriverStation.reportError("RECORDER FLUSHED", false);
    } catch (IOException ex) {
      DriverStation.reportWarning("IO EXCEPTION OCCURRED", true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}