package frc.robot;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.google.common.collect.ImmutableMap;

public enum State {
    IDLE,
    
    INTAKING_WITH_INDEXER,
    LAUNCHING_WITH_INDEXER;

    private State() {
    }

    // Define the scoring sequence map, the 2nd state is the next state after the
    // current one
    // private static final ImmutableMap<State, State> scoreSequence = ImmutableMap.ofEntries();

    // ------ STATE ATTRIBUTES ------//

    // ------------------------------//
}
