package frc.robot.util.choosers;

import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Dep.DepCycle1;
import frc.robot.Auton.Dep.DepToSSTrench1;
import frc.robot.Auton.Out.OutToSSTrench1;
import frc.robot.Auton.Out.OutCycle1;
import frc.robot.Auton.Hub.HubCycle1;
import frc.robot.Auton.Forward2m;
import frc.robot.Auton.OneAuton;
import frc.robot.RobotContainer;

public class AutonomousChooser extends ChooserBase<AutonomousChooser.AutonomousMode> {
  public AutonomousChooser() {
    super("Autonomous Mode");

    setDefaultOption(AutonomousMode.ONE_AUTON)
        .addOption(AutonomousMode.TEST_FORWARD)
        .addOption(AutonomousMode.DEP_CYCLE1)
        .addOption(AutonomousMode.DEPSSTRENCH1)
        .addOption(AutonomousMode.OUT_CYCLE1)
        .addOption(AutonomousMode.OUTSSTRENCH1)
        .addOption(AutonomousMode.HUB_CYCLE1);
  }

  public AutonCommandBase getCommand() {
    return getSelectedMode().getCommand();
  }

  public AutonomousMode getSelectedMode() {
    AutonomousMode selected = getSendable().getSelected();
    return selected != null ? selected : AutonomousMode.TEST_FORWARD;
  }

  public enum AutonomousMode {
    ONE_AUTON("one cycle anywhere"),
    TEST_FORWARD("test forward"),
    DEP_CYCLE1("depo cycle"),
    DEPSSTRENCH1("depo to ss trench1"),
    OUT_CYCLE1("out cycle"),
    OUTSSTRENCH1("out to ss trench1"),
    HUB_CYCLE1("hub cycle");

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
        case DEP_CYCLE1:
          return new DepCycle1(RobotContainer.getInstance());
        case DEPSSTRENCH1:
          return new DepToSSTrench1(RobotContainer.getInstance());
        case OUT_CYCLE1:
        return new OutCycle1(RobotContainer.getInstance());
        case OUTSSTRENCH1:
          return new OutToSSTrench1(RobotContainer.getInstance());
        case HUB_CYCLE1:
          return new HubCycle1(RobotContainer.getInstance());
        default:
          return new OneAuton(RobotContainer.getInstance());
      }
    }

    public boolean requiresPathLoading() {
      return this != TEST_FORWARD;
    }

    public boolean disablesVisionSeeding() {
      return this == TEST_FORWARD;
    }
  }
}
