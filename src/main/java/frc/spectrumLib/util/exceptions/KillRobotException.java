package frc.spectrumLib.util.exceptions;

/** Thrown to stop robot code when continuing would be unsafe. */
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
