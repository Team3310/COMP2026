package frc.lib.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.ArrayList;
import java.util.List;

/**
 * Centralises CAN signal refreshes so that every {@link TalonFXIO} in the robot has its status
 * signals refreshed in a single {@code BaseStatusSignal.refreshAll()} call per cycle instead of one
 * call per motor.
 *
 * <p>Calling {@code refreshAll()} once with <i>all</i> signals is dramatically cheaper than calling
 * it 13+ times with 6 signals each, because the Phoenix 6 API batches the CAN frames internally. On
 * a roboRIO with ~13 Talon FX motors this change alone can save 5-15 ms per cycle.
 *
 * <h3>Usage</h3>
 *
 * <ol>
 *   <li>Each {@code TalonFXIO} constructor calls {@link #register(BaseStatusSignal[])} to add its
 *       signals.
 *   <li>At the very top of the main robot loop (before any subsystem {@code periodic()} runs), call
 *       {@link #refreshAll()}.
 *   <li>{@code TalonFXIO.readInputs()} no longer calls {@code BaseStatusSignal.refreshAll()} — it
 *       just reads the already-refreshed cached values.
 * </ol>
 */
public final class SignalRefreshManager {
  private static final SignalRefreshManager INSTANCE = new SignalRefreshManager();

  private final List<BaseStatusSignal> signalList = new ArrayList<>();
  private BaseStatusSignal[] signalArray = new BaseStatusSignal[0];
  private boolean dirty = true;

  private SignalRefreshManager() {}

  public static SignalRefreshManager getInstance() {
    return INSTANCE;
  }

  /**
   * Registers an array of status signals (typically the 6 per-motor signals from a {@link
   * TalonFXIO}) so they will be included in the next batch refresh.
   */
  public void register(BaseStatusSignal[] signals) {
    for (BaseStatusSignal sig : signals) {
      signalList.add(sig);
    }
    dirty = true;
  }

  /**
   * Refreshes <b>all</b> registered status signals in a single CAN transaction. Call this once per
   * robot loop iteration, before any subsystem reads its inputs.
   */
  public void refreshAll() {
    if (signalList.isEmpty()) return;
    if (dirty) {
      signalArray = signalList.toArray(new BaseStatusSignal[0]);
      dirty = false;
    }
    BaseStatusSignal.refreshAll(signalArray);
  }
}
