package frc.lib.math;

/**
 * This class uses a double to interpolate between Interpolable<T>
 *
 * @param <T>
 *            Class that implements Interpolable<T>
 */

public class InterpolablePair<T extends Interpolable<T>> {

	private T value;
	private double key;

	public InterpolablePair(double key, T value) {
		this.key = key;
		this.value = value;
	}

	public double getKey() {
		return key;
	}

	public T getValue() {
		return value;
	}
}
