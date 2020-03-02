package frc.lib.math;

/**
 * Stores a cos and sin that is used like a rotation matrix
 */
public class Rotation implements Interpolable<Rotation> {

	/**
	 * Gets a Rotation from a specified degree
	 *
	 * @param angle
	 *            Angle to turn into a rotation matrix
	 * @return Rotation from specified angle in argument
	 */
	public static Rotation fromDegrees(double angle) {
		return Rotation.fromRadians(Math.toRadians(angle));
	}

	/**
	 * Gets a Rotation matrix from a specified radian
	 *
	 * @param radians
	 *            Radian to turn into a rotation matrix
	 * @return Rotation from specified radian in argument
	 */
	public static Rotation fromRadians(double radians) {
		return new Rotation(Math.cos(radians), Math.sin(radians));
	}

	private double cos;

	private double sin;

	public Rotation() {
		cos = 1;
		sin = 0;
	}

	public Rotation(double cos, double sin) {
		this.cos = cos;
		this.sin = sin;
	}

	public Rotation(double cos, double sin, boolean normalize) {
		this.cos = cos;
		this.sin = sin;
		if (normalize) {
			normalize();
		}
	}

	/**
	 *
	 * @return The cosine of this Rotation
	 */
	public double cos() {
		return cos;
	}

	/**
	 *
	 * @return The rotation matrix in degrees
	 */
	public double getDegrees() {
		return Math.toDegrees(getRadians());
	}

	/**
	 *
	 * @return The rotation matrix in radians
	 */
	public double getRadians() {
		return Math.atan2(sin, cos);
	}

	// TODO: make it work
	@Override
	public Rotation interpolate(Rotation other, double percentage) {
		Rotation diff = inverse().rotateBy(other);
		return rotateBy(Rotation.fromRadians(diff.getRadians() * percentage));
	}

	/**
	 *
	 * @return The Rotation that when rotated with this Rotation moves the cos
	 *         to 1 and the sin to 0
	 */
	public Rotation inverse() {
		return new Rotation(cos, -sin);
	}

	/**
	 *
	 * @return The Rotation that is flipped about the x and y axis
	 */
	public Rotation flip() {
		return new Rotation(-cos, -sin);
	}

	/**
	 * Makes magnitude 1
	 */
	public void normalize() {
		double magnitude = Math.hypot(cos, sin);
		if (magnitude > 1E-9) {
			cos /= magnitude;
			sin /= magnitude;
		} else {
			cos = 1;
			sin = 0;
		}
	}

	/**
	 *
	 * @param rotationMat
	 *            Multiply this Rotation by this specified Rotation
	 * @return The multiplied Rotation
	 */
	public Rotation rotateBy(Rotation rotationMat) {
		return new Rotation(cos * rotationMat.cos() - sin * rotationMat.sin(),
				sin * rotationMat.cos() + cos * rotationMat.sin(), true);
	}

	/**
	 *
	 * @return The sin of this Rotation
	 */
	public double sin() {
		return sin;
	}
}