package frc.lib.math;

/**
 * Stores an x and y value. Rotates and Translates of objects returns a new
 * object.
 */
public class Translation2d implements Interpolable<Translation2d> {

	public static Translation2d fromAngleDistance(double distance, Rotation angle) {
		return new Translation2d(angle.sin() * distance, angle.cos() * distance);
	}

	private double x;
	private double y;

	public Translation2d() {
		x = 0;
		y = 0;
	}

	public Translation2d(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * Get the angle from offset to this Translation2d. This is done by making
	 * the offset the origin and finding the angle to this point. Then the angle
	 * to the Y axis is found from this angle. The points are treated as a
	 * vector and the direction is taken off from it. The coordinates of the
	 * points correspond to the unit circle.
	 *
	 * @param offset
	 *            Point that becomes the new origin for the other point
	 * @return Angle to the Y axis from the angle from the offset to this point
	 */
	public Rotation getAngleFromOffsetFromYAxis(Translation2d offset) {
		return offset.getAngleFromYAxis(this);
	}

	/**
	 * Get the angle from offset to this Translation2d. This is done by making
	 * the offset the origin and finding the angle to this point. The points are
	 * treated as a vector and the direction is taken off from it. The
	 * coordinates of the points correspond to the unit circle.
	 *
	 * @param offset
	 *            Point that becomes the origin for the other point
	 * @return Angle from the offset to this point
	 */
	public Rotation getAngleFromOffset(Translation2d offset) {
		return offset.getAngle(this);
	}

	/**
	 * Get the angle from this point to another point. Then the angle to the Y
	 * axis is found from this angle. The points are treated as a vector and the
	 * direction is taken off from it. The coordinates of the points correspond
	 * to the unit circle.
	 *
	 * @param nextPoint
	 *            Point to find angle to from this point
	 * @return Angle of the two points.
	 */
	public Rotation getAngleFromYAxis(Translation2d nextPoint) {
		double angleOffset = Math.asin((x - nextPoint.getX()) / getDistanceTo(nextPoint));
		return Rotation.fromRadians(angleOffset);
	}

	/**
	 * Get the angle from this point to another point. The points are treated as
	 * a vector and the direction is taken off from it. The coordinates of the
	 * points correspond to the unit circle.
	 *
	 * @param nextPoint
	 *            Point to find angle to from this point.
	 * @return Angle of the two points.
	 */
	public Rotation getAngle(Translation2d nextPoint) {
		double angleOffset = Math.atan2(nextPoint.getY() - y, nextPoint.getX() - x);
		return Rotation.fromRadians(angleOffset);
	}

	/**
	 * Get the distance between the this point and the point specified in the
	 * argument.
	 *
	 * @param nextPoint
	 *            Point to find distance to.
	 * @return Distance between this point and the specified point.
	 */
	public double getDistanceTo(Translation2d nextPoint) {
		return Math.sqrt(Math.pow((x - nextPoint.getX()), 2) + Math.pow(y - nextPoint.getY(), 2));
	}

	/**
	 *
	 * @return X value of this object.
	 */
	public double getX() {
		return x;
	}

	/**
	 *
	 * @return Y value of this object.
	 */
	public double getY() {
		return y;
	}

	/**
	 *
	 * @return Returns a Translation2d that when translated with this
	 *         Translation2d becomes 0, 0. Essentially the negative x and y of
	 *         this Translation2d.
	 */
	public Translation2d inverse() {
		return new Translation2d(-x, -y);
	}

	/**
	 * Multiplies this point with a specified rotation matrix
	 *
	 * @param rotationMat
	 *            Rotation matrix to multiply point with
	 * @return Rotated point
	 */
	public Translation2d rotateBy(Rotation rotationMat) {
		double x2 = x * rotationMat.cos() - y * rotationMat.sin();
		double y2 = x * rotationMat.sin() + y * rotationMat.cos();
		return new Translation2d(x2, y2);
	}

	/**
	 * Translation this point by another point.
	 *
	 * @param delta
	 *            Translation2d to change this point by
	 * @return Translated point
	 */
	public Translation2d translateBy(Translation2d delta) {
		return new Translation2d(x + delta.getX(), y + delta.getY());
	}

	@Override
	public Translation2d interpolate(Translation2d other, double percentage) {
		Translation2d delta = new Translation2d(this.getX() - other.getX(), this.getY() - other.getY());
		return new Translation2d(this.getX() + delta.getX() * percentage, this.getY() + delta.getY() * percentage);
	}
}