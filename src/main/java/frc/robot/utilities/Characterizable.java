package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SysIdRoutines.SysIdType;

public interface Characterizable {
  public String getName();

  public SysIdRoutine getRoutine(SysIdType routineType);
}
