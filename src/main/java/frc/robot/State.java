package frc.robot;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.google.common.collect.ImmutableMap;

public enum State {
    IDLE,

    INTAKE_FUEL,

    TURRET_TRACK_WITH_SPINUP,
    TURRET_TRACK_WITH_LAUNCH,

    TURRET_FEED_WITH_SPINUP,
    TURRET_FEED_WITH_LAUNCH,

    L1_CLIMB_PREP,
    L1_CLIMB_EXECUTE,

    L3_CLIMB_PREP,
    L3_CLIMB_EXECUTE;

    private State() {
    }

    // Define the scoring sequence map, the 2nd state is the next state after the
    // current one
    private static final ImmutableMap<State, State> scoreSequence = ImmutableMap.ofEntries(
            Map.entry(TURRET_TRACK_WITH_SPINUP, TURRET_TRACK_WITH_LAUNCH));

    // ------ STATE ATTRIBUTES ------//

    public State getNextState(State state) {
        return scoreSequence.getOrDefault(state, state);
    }

    private static BooleanSupplier isReadyState(State state) {
        return () -> switch (state) {
            case TURRET_TRACK_WITH_SPINUP -> true;
            default -> false;
        };
    }

    public BooleanSupplier isReady() {
        return isReadyState(this);
    }

    public State getNext() {
        return getNextState(this);
    }
}
