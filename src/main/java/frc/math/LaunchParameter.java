package frc.quixlib.math;

import edu.wpi.first.math.interpolation.Interpolatable;

public class LaunchParameter implements Interpolatable<LaunchParameter> {
  private boolean m_inTableRange = true;
  private final double m_launchVelocityMps;
  private final double m_launchAngleDeg;

  public LaunchParameter(double launchVelocityMps, double launchAngleDeg) {
    this.m_launchVelocityMps = launchVelocityMps;
    this.m_launchAngleDeg = launchAngleDeg;
  }

  public boolean inTableRange() {
    return m_inTableRange;
  }

  public LaunchParameter updateInTableRange(boolean inTableRange) {
    m_inTableRange = inTableRange;
    return this;
  }

  public double launchVelocityMps() {
    return m_launchVelocityMps;
  }

  public double launchAngleDeg() {
    return m_launchAngleDeg;
  }

  public LaunchParameter interpolate(LaunchParameter endValue, double t) {
    if (t < 0) {
      return this;
    } else if (t >= 1) {
      return endValue;
    } else {
      return new LaunchParameter(
          (1 - t) * this.m_launchVelocityMps + t * endValue.m_launchVelocityMps,
          (1 - t) * this.m_launchAngleDeg + t * endValue.m_launchAngleDeg);
    }
  }
}
