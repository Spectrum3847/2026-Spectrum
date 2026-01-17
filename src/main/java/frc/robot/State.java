package frc.robot;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.google.common.collect.ImmutableMap;

public enum State {
    REHOME,

    IDLE,
    
    INTAKE,

    LAUNCH_READY,
    LAUNCH_FINISH,
    
    CLIMBING_HANG,
    CLIMBING_LOCK;


    private State() {}

    // Define the scoring sequence map, the 2nd state is the next state after the current one
    private static final ImmutableMap<State, State> scoreSequence = 
        ImmutableMap.ofEntries(
            Map.entry(LAUNCH_READY, LAUNCH_FINISH)
        );

    // ------ STATE ATTRIBUTES ------//

    public State getNextState(State state) {
        return scoreSequence.getOrDefault(state, state);
    }

    private static BooleanSupplier isReadyState(State state) {
        return () ->
                switch (state) {
                    case LAUNCH_READY -> true;
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
