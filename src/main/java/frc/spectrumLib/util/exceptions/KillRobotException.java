package frc.spectrumLib.util.exceptions;

/**
 * Unchecked exception for unrecoverable states where the robot must stop rather than keep running.
 */
public class KillRobotException extends RuntimeException {

    public KillRobotException(String message) {
        super(message);
    }

    public KillRobotException(Throwable cause) {
        super(cause);
    }

    public KillRobotException(String message, Throwable throwable) {
        super(message, throwable);
    }
}
