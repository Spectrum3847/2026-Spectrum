package frc.robot;

import com.google.common.collect.ImmutableMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public enum State {
    IDLE,

    INTAKE_FUEL,
    SNAKE_INTAKE,

    TURRET_TRACK,
    TURRET_TRACK_WITH_LAUNCH,

    TURRET_WITHOUT_TRACK,
    TURRET_WITHOUT_TRACK_WITH_LAUNCH,

    TURRET_FEED_WITH_AIMING,
    TURRET_FEED_WITH_LAUNCH,

    TURRET_FEED_WITHOUT_AIMING,
    TURRET_FEED_WITHOUT_AIMING_WITH_LAUNCH,

    L1_CLIMB_PREP,
    L1_CLIMB_EXECUTE,

    L3_CLIMB_PREP,
    L3_CLIMB_EXECUTE;

    private State() {}

    // Define the scoring sequence map, the 2nd state is the next state after the
    // current one
    private static final ImmutableMap<State, State> scoreSequence =
            ImmutableMap.ofEntries(Map.entry(TURRET_TRACK, TURRET_TRACK_WITH_LAUNCH));

    // ------ STATE ATTRIBUTES ------//

    public State getNextState(State state) {
        return scoreSequence.getOrDefault(state, state);
    }

    private static BooleanSupplier isReadyState(State state) {
        return () ->
                switch (state) {
                    case TURRET_TRACK -> true;
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
