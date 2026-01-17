package frc.robot;

public enum State {
    IDLE,

    INTAKE_FUEL,

    AIM_TURRET_WITH_SPINUP,
    AIM_TURRET_WITH_LAUNCH,

    FIX_TURRET_WITH_SPINUP,
    FIX_TURRET_WITH_LAUNCH,

    L1_CLIMB_PREP,
    L1_CLIMB_EXECUTE,
    
    L3_CLIMB_PREP,
    L3_CLIMB_EXECUTE;

    private State() {}

    // Define the scoring sequence map, the 2nd state is the next state after the current one
    // private static final ImmutableMap<State, State> scoreSequence =

    // ------ STATE ATTRIBUTES ------//
    
    // ------------------------------//

    // public State getNextState(State state) {
    //     return scoreSequence.getOrDefault(state, state);
    // }

    // public State getNext() {
    //     return getNextState(this);
    // }
}
