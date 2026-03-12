package frc.robot;

import com.google.common.collect.ImmutableMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public enum State {
    IDLE,
    FORCE_HOME,

    INTAKE_FUEL,
    SNAKE_INTAKE,

    TURRET_TRACK,
    TURRET_TRACK_WITH_LAUNCH,
    CUSTOM_SPEED_TURRET_LAUNCH,

    UNJAM,

    COAST,
    BRAKE,

    TEST_INFINITE_LAUNCH,
    TEST_IDLE;

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
