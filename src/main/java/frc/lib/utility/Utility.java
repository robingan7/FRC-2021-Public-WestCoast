package frc.lib.utility;

public class Utility {
	public static final double epsilon = 1E-5;

	/**
	 * Keeps a value in a range by truncating it.
	 *
	 * @param toCoerce
	 *            the value to clamp
	 * @param high
	 *            the high value of the range
	 * @param low
	 *            the low value of the range
	 * @return the coerced value
	 */
	public static double clamp(double toCoerce, double high, double low) {
		if (toCoerce > high) {
			return high;
		} else if (toCoerce < low) {
			return low;
		}
		return toCoerce;
	}

	public static boolean doubleEqual(double a, double b) {
		double epsilon = 1E-14;
		return Math.abs(a - b) < epsilon;
	}

	public static boolean doubleEqual(double a, double b, double epsilon) {
		return Math.abs(a - b) < epsilon;
	}

	public static double normalize(double toNormalize, double fromHigh, double fromLow, double toHigh, double toLow) {
		double factor = (toHigh - toLow) / (fromHigh - fromLow);
		double add = toLow - fromLow * factor;
		return toNormalize * factor + add;
	}

	public static double clampThenNormalize(double rawValue, double minInput, double maxInput, double minOutput,
			double maxOutput) {
		if (rawValue < minInput) {
			return minOutput;
		} else if (rawValue > maxInput) {
			return maxOutput;
		}
		double norm = (Math.abs(rawValue) - minInput) / (maxInput - minInput);
		norm = Math.copySign(norm * (maxOutput - minOutput), rawValue) + minOutput;
		return norm;
	}

	/**
	 * Encapsulates Thread.sleep to make code more readable.
	 *
	 * @param millis
	 *            the time to sleep
	 */
	public static void sleep(long millis) {
		try {
			Thread.sleep(millis);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	/**
	 * set angle range to -180 and 180
	 */
	public static double setAngleRange(double angleIn, double wrapAroundAngle) {
		if(angleIn > wrapAroundAngle) {
			return -(360 - angleIn);
		} else if(angleIn < (wrapAroundAngle - 360)) {
			return 360 + angleIn;
		}

		return angleIn;
	}

	public static double setAngleRange(double angleIn) {
		if(angleIn < -100) {
			return 360 + angleIn;
		} else if(angleIn > 260) {
			return -(360 - angleIn);
		}

		return angleIn;
	}

	public static double meterToInch(double meters) {
		return meters / 0.0254;
	}

	public static boolean isReachSpeedShooter(double current, double goal) {
		return Math.abs(current - goal) < 0.1;
	}

	public static boolean isCloseEnough(double valueIn, double targetValue, double tolerate) {
		return Math.abs(targetValue - valueIn) < tolerate;
	}

}
