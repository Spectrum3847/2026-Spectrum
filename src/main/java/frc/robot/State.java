package frc.robot;

public enum State {
    IDLE,

    CLIMB_FORWARD,
    CLIMB_BACKWARD;

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
