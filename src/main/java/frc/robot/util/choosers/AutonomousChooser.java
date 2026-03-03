package frc.robot.util.choosers;

import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Dep.DepLoop1;
import frc.robot.Auton.Forward2m;
import frc.robot.Auton.OneAuton;
import frc.robot.RobotContainer;

public class AutonomousChooser extends ChooserBase<AutonomousChooser.AutonomousMode> {
  public AutonomousChooser() {
    super("Autonomous Mode");

    setDefaultOption(AutonomousMode.ONE_AUTON)
        .addOption(AutonomousMode.TEST_FORWARD)
        .addOption(AutonomousMode.DEP_LOOP);
  }

  public AutonCommandBase getCommand() {
    return getSendable().getSelected().getCommand();
  }

  public enum AutonomousMode {
    ONE_AUTON("one cycle anywhere"),
    TEST_FORWARD("test forward"),

    DEP_LOOP("depo loop");

    private String name = "";

    private AutonomousMode(String name) {
      this.name = name;
    }

    @Override
    public String toString() {
      return name;
    }

    public AutonCommandBase getCommand() {
      switch (this) {
        case TEST_FORWARD:
          return new Forward2m(RobotContainer.getInstance());
        case DEP_LOOP:
          return new DepLoop1(RobotContainer.getInstance());
        default:
          return new OneAuton(RobotContainer.getInstance());
      }
    }
  }
}
