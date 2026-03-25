package frc.robot.util.choosers;

import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Dep.DepCollect;
import frc.robot.Auton.Dep.DepSnowblowMidShort2;
import frc.robot.Auton.OneAuton;
import frc.robot.Auton.Outpost.CitrusAuton;
import frc.robot.Auton.Outpost.OutCollectMidShort2;
import frc.robot.Auton.Outpost.OutSnowblowMidShort;
import frc.robot.Auton.Outpost.OutSnowblowMidShort2;
import frc.robot.RobotContainer;

public class AutonomousChooser extends ChooserBase<AutonomousChooser.AutonomousMode> {
  public AutonomousChooser() {
    super("Autonomous Mode");

    setDefaultOption(AutonomousMode.ONE_AUTON)
        // .addOption(AutonomousMode.TEST_FORWARD)
        // .addOption(AutonomousMode.DEP_CYCLE1)
        // .addOption(AutonomousMode.DEPSSTRENCH1)
        // .addOption(AutonomousMode.OUT_CYCLE1)
        // .addOption(AutonomousMode.OUTSSTRENCH1)
        // .addOption(AutonomousMode.HUB_CYCLE1)
        // .addOption(AutonomousMode.DEP_TO_DEP)
        // .addOption(AutonomousMode.DEP_TO_OUT)
        // .addOption(AutonomousMode.OUT_TO_DEP)
        // .addOption(AutonomousMode.OUT_TO_OUT)
        // .addOption(AutonomousMode.HUB_TO_OUT)
        // .addOption(AutonomousMode.HUB_TO_DEP)
        // .addOption(AutonomousMode.DEP_SNOWBLOW_MID_SHORT)
        .addOption(AutonomousMode.DEP_COLLECT)
        // .addOption(AutonomousMode.OUT_SNOWBLOW_MID_SHORT)
        // .addOption(AutonomousMode.OUT_SNOWBLOW_MID_SHORT2)
        .addOption(AutonomousMode.OUT_COLLECT_MID_SHORT2)
        .addOption(AutonomousMode.CITRUS_AUTON);
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
    // DEP_CYCLE1("depo cycle"),
    // DEPSSTRENCH1("depo to ss trench1"),
    // OUT_CYCLE1("out cycle"),
    // OUTSSTRENCH1("out to ss trench1"),
    // HUB_CYCLE1("hub cycle"),
    // DEP_TO_DEP("depo to depo"),
    // DEP_TO_OUT("depo to out"),
    // OUT_TO_DEP("out to depo"),
    // OUT_TO_OUT("out to out"),
    // HUB_TO_OUT("hub to out"),
    // HUB_TO_DEP("hub to depo"),
    DEP_SNOWBLOW_MID_SHORT("depo snowblow mid short"),
    DEP_COLLECT("DepCollect"),

    OUT_SNOWBLOW_MID_SHORT("out snowblow mid short"),
    OUT_SNOWBLOW_MID_SHORT2("2out snowblow mid short"),
    OUT_COLLECT_MID_SHORT2("2out collect mid short"),
    CITRUS_AUTON("citrus auton");

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
          // case TEST_FORWARD:
          //   return new Forward2m(RobotContainer.getInstance());
          // case DEP_CYCLE1:
          //   return new DepCycle1(RobotContainer.getInstance());
          // case DEPSSTRENCH1:
          //   return new DepToSSTrench1(RobotContainer.getInstance());
          // case OUT_CYCLE1:
          //   return new OutCycle1(RobotContainer.getInstance());
          // case OUTSSTRENCH1:
          //   return new OutToSSTrench1(RobotContainer.getInstance());
          // case HUB_CYCLE1:
          //   return new HubCycle1(RobotContainer.getInstance());
          // case DEP_TO_DEP:
          //   return new DepToDep(RobotContainer.getInstance());
          // case DEP_TO_OUT:
          //   return new DepToOut(RobotContainer.getInstance());
          // case OUT_TO_DEP:
          //   return new OutToDep(RobotContainer.getInstance());
          // case OUT_TO_OUT:
          //   return new OutToOut(RobotContainer.getInstance());
          // case HUB_TO_OUT:
          //   return new HubToOut(RobotContainer.getInstance());
          // case HUB_TO_DEP:
          //   return new HubToDep(RobotContainer.getInstance());
        case DEP_COLLECT:
          return new DepCollect(RobotContainer.getInstance());
        case DEP_SNOWBLOW_MID_SHORT:
          return new DepSnowblowMidShort2(RobotContainer.getInstance());
        case OUT_SNOWBLOW_MID_SHORT:
          return new OutSnowblowMidShort(RobotContainer.getInstance());
        case OUT_SNOWBLOW_MID_SHORT2:
          return new OutSnowblowMidShort2(RobotContainer.getInstance());
        case OUT_COLLECT_MID_SHORT2:
          return new OutCollectMidShort2(RobotContainer.getInstance());
        case CITRUS_AUTON:
          return new CitrusAuton(RobotContainer.getInstance());
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
