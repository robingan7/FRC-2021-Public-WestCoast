package frc.lib.control;

/**
 * Controller that adds a specific amount of extra load for rising edges that
 * decays linearly to 0.
 */

public class LoadController {

	private double loadAccum;
	private double loadIncrease;
	private double decayRate;
	private double baseF;

	private boolean shoot = false;
	private boolean lastShoot = false;

	/**
	 *
	 * @param baseF
	 *            Feed forward
	 * @param loadIncrease
	 *            Amount of load to increase for each rising edge
	 * @param decayRate
	 *            Amount of load to decrease for each iteration
	 */
	public LoadController(double baseF, double loadIncrease, double decayRate) {
		this.baseF = baseF;
		this.loadIncrease = loadIncrease;
		this.decayRate = decayRate;
	}

	/**
	 *
	 * @param input
	 *            True if adding load and false if not
	 * @param setpoint
	 *            Setpoint that feed forward uses to calculate output
	 * @return
	 */
	public double calculate(boolean input, double setpoint) {
		lastShoot = shoot;
		shoot = input;
		loadAccum = loadAccum - decayRate;

		if (loadAccum < 0) {
			loadAccum = 0;
		}

		if (shoot && !lastShoot) {
			loadAccum += loadIncrease;
		}

		double correctedOutput = baseF * setpoint + loadAccum;
		if (setpoint == 0) {
			return 0;
		}
		return correctedOutput;
	}

	/**
	 *
	 * @return Current feed forward coefficient
	 */
	public double getBaseF() {
		return baseF;
	}

	/**
	 *
	 * @param baseF
	 *            Feed forward coefficient to set controller to
	 */
	public void setBaseF(double baseF) {
		this.baseF = baseF;
	}

	/**
	 *
	 * @return Current decay rate
	 */
	public double getDecayRate() {
		return decayRate;
	}

	/**
	 *
	 * @param decayRate
	 *            Decay rate to set controller to
	 */
	public void setDecayRate(double decayRate) {
		this.decayRate = decayRate;
	}

	/**
	 *
	 * @return Current load increase
	 */
	public double getLoadIncrease() {
		return loadIncrease;
	}

	/**
	 *
	 * @param loadIncrease
	 *            Load increase to set controller to
	 */
	public void setLoadIncrease(double loadIncrease) {
		this.loadIncrease = loadIncrease;
	}
}
