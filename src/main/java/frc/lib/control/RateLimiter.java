package frc.lib.control;

import frc.lib.utility.Utility;

import edu.wpi.first.wpilibj.Timer;

/**
 * Limits acceleration and optionally jerk
 */
public class RateLimiter {

	private double maxAccel, maxJerk, latestValue;
	private double accValue;
	private double lastTime;

	public RateLimiter(double accel) {
		this(accel, Double.POSITIVE_INFINITY);
	}

	/**
	 *
	 * @param accel
	 *            Maximum acceleration in units with an arbitrary time unit. The
	 *            units match whatever you send in update()
	 * @param jerk
	 *            Maximum jerk in units with an arbitrary time unit. The units
	 *            match whatever you send in update()
	 */
	public RateLimiter(double accel, double jerk) {
		this.maxAccel = accel;
		this.maxJerk = jerk;
		latestValue = 0;
	}

	/**
	 *
	 * @param setpoint
	 *            What value to accelerate towards
	 * @param dt
	 *            How much time has past between iterations
	 * @return Calculated latest value
	 */
	public double update(double setpoint) {
		double dt = Timer.getFPGATimestamp() - lastTime;
		lastTime = Timer.getFPGATimestamp();
		double diff = setpoint - latestValue;
		if (diff == 0) {
			return latestValue;
		}
		double area = (Math.pow(accValue, 2) / maxJerk);// Trapezoidal
														// acceleration area at
														// decrease in jerk ->
														// total velocity
		/*
		 * Check if we need to start decelerating. Area does not have a sign so
		 * check sign if accValue is towards setpoint or away or else it will
		 * keep "decelerating" away from the setpoint.
		 */
		if (Math.abs(diff) >= area || Math.signum(diff) != Math.signum(accValue)) {
			accValue = accValue + Math.copySign(maxJerk * dt, diff);
			/*
			 * Limit accValue to the maximum acceleration
			 */
			accValue = Utility.clamp(accValue, maxAccel, -maxAccel);
			latestValue += accValue * dt;
		} else {
			accValue = accValue - Math.copySign(maxJerk * dt, diff);
			latestValue += accValue * dt;
		}
		/*
		 * Makes sure latestValue isn't greater than setpoint. accValue is moved
		 * to 0 because the else part of the statement above changes the
		 * accValue without despite it not needing to be changed.
		 */
		if (Math.signum(setpoint - latestValue) != Math.signum(diff)) {
			latestValue = setpoint;
			accValue = 0;
		}
		return latestValue;
	}

	/**
	 *
	 * @param setpoint
	 *            What value to accelerate towards
	 * @param dt
	 *            How much time has past between iterations
	 * @param remainingDist
	 *            Distance remaining before complete stop
	 * @return Calculated latest value
	 */
	public double update(double setpoint, double remainingDist) {
		// double timeToSwitchAcc = (getAcc() / getMaxJerk());
		double timeToDecel = getLatestValue() / getMaxAccel();
		double distanceTillStop = timeToDecel * getLatestValue();
		if (Math.abs(distanceTillStop) >= remainingDist) {
			return update(0.0);
		} else {
			return update(setpoint);
		}
	}

	/**
	 *
	 * @return Current acceleration value
	 */
	public double getAcc() {
		return accValue;
	}

	/**
	 *
	 * @return Current maximum jerk value
	 */
	public double getMaxJerk() {
		return maxJerk;
	}

	/**
	 *
	 * @return Current maximum acceleration value
	 */
	public double getMaxAccel() {
		return maxAccel;
	}

	/**
	 *
	 * @return Latest value calculated
	 */
	public double getLatestValue() {
		return latestValue;
	}

	/**
	 *
	 * @param val
	 *            Value to set the latest value to
	 */
	public void setLatestValue(double val) {
		latestValue = val;
	}

	/**
	 * Sets the latest value to 0, current acceleration value to 0 and time
	 */
	public void reset() {
		latestValue = 0;
		accValue = 0;
		lastTime = Timer.getFPGATimestamp();
	}

	public void resetTime() {
		lastTime = Timer.getFPGATimestamp();
	}

	/**
	 *
	 * @param maxAccel
	 *            Wanted max acceleration
	 */
	public void setMaxAccel(double maxAccel) {
		this.maxAccel = maxAccel;
		accValue = Utility.clamp(accValue, maxAccel, -maxAccel);
	}

	/**
	 *
	 * @param maxJerk
	 *            Wanted max Jerk
	 */
	public void setMaxJerk(double maxJerk) {
		this.maxJerk = maxJerk;
	}

	/**
	 *
	 * @param accValue
	 *            Wanted acceleration value
	 */
	public void setAccValue(double accValue) {
		this.accValue = accValue;
	}
}
