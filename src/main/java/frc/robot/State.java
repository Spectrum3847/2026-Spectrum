package frc.robot;

import com.google.common.collect.ImmutableMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public enum State {
    IDLE,

    INTAKE_FUEL,
    SNAKE_INTAKE,

    TRACK_TARGET,
    TRACK_TARGET_WITH_NO_SWERVE,
    LAUNCH_WITH_SQUEEZE,
    LAUNCH_WITHOUT_SQUEEZE,

    AUTON_TRACK_TARGET,
    AUTON_LAUNCH_WITH_SQUEEZE,

    CUSTOM_SPEED_TURRET_LAUNCH,
    UNJAM,
    FORCE_HOME,

    COAST,
    BRAKE,
    TEST_INFINITE_LAUNCH,
    TEST_IDLE;

    private State() {}

    // Define the scoring sequence map, the 2nd state is the next state after the
    // current one
    private static final ImmutableMap<State, State> scoreSequence =
            ImmutableMap.ofEntries(Map.entry(TRACK_TARGET, LAUNCH_WITH_SQUEEZE));

    // ------ STATE ATTRIBUTES ------//

    public State getNextState(State state) {
        return scoreSequence.getOrDefault(state, state);
    }

    private static BooleanSupplier isReadyState(State state) {
        return () ->
                switch (state) {
                    case TRACK_TARGET -> true;
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
