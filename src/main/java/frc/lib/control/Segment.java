package frc.lib.control;

import frc.lib.math.Translation2d;

/**
 * Two points that function as a line.
 */
public class Segment {

	private Translation2d start, end, delta;
	private double maxSpeed, deltaDist, deltaDistSquared;
	

	public Segment(double xStart, double yStart, double xEnd, double yEnd, double maxSpeed) {
		this(new Translation2d(xStart, yStart), new Translation2d(xEnd, yEnd), maxSpeed);
	}

	public Segment(Translation2d start, Translation2d end, double maxSpeed) {
		this.start = start;
		this.end = end;
		this.maxSpeed = maxSpeed;
		delta = start.inverse().translateBy(end);
		deltaDist = Math.hypot(delta.getX(), delta.getY());
		deltaDistSquared = Math.pow(deltaDist, 2);
	}

	public Translation2d getStart() {
		return start;
	}

	public Translation2d getEnd() {
		return end;
	}

	/**
	 * Gets the point on the line closest to the point defined in the
	 * argument
	 *
	 * @param point
	 *            Point to find the closest point to
	 * @return The closest point on the line to the point specified
	 */
	public Translation2d getClosestPoint(Translation2d point) {
		double u = ((point.getX() - start.getX()) * delta.getX() + (point.getY() - start.getY()) * delta.getY())
				/ deltaDistSquared;
		u = Math.max(Math.min(u, 1), 0);
		return new Translation2d(start.getX() + delta.getX() * u, start.getY() + delta.getY() * u);
	}
	
	public double getPercentageOnSegment(Translation2d point) {
		double u = ((point.getX() - start.getX()) * delta.getX() + (point.getY() - start.getY()) * delta.getY())
				/ deltaDistSquared;
		u = Math.max(Math.min(u, 1), 0);
		return u;
	}

	/**
	 * Returns the point on the line which is some distance away from the
	 * start. The point travels on the line towards the end. More efficient
	 * than calling interpolate in Translation2d because delta is
	 * pre-computed before.
	 *
	 * @param distance
	 *            Distance from the start of the line.
	 * @return Point on the line that is the distance away specified.
	 */
	public Translation2d getPointByDistance(double distance) {
		distance = Math.pow(distance, 2);
		double u = Math.sqrt(distance / deltaDistSquared);
		return new Translation2d(start.getX() + delta.getX() * u, start.getY() + delta.getY() * u);
	}
	
	public Translation2d getPointByPercentage(double percentage) {
		return new Translation2d(start.getX() + delta.getX() * percentage, start.getY() + delta.getY() * percentage);
	}

	/**
	 * Returns the point on the line which is some distance away from the
	 * end. The point travels on the line towards the start.
	 *
	 * @param distance
	 *            Distance from the end of the line.
	 * @return Point on the line that is the distance away from the end
	 */
	public Translation2d getPointByDistanceFromEnd(double distance) {
		distance = Math.pow(distance, 2);
		double u = Math.sqrt(distance / deltaDistSquared);
		return new Translation2d(end.getX() - delta.getX() * u, end.getY() - end.getY() * u);
	}

	/**
	 *
	 * @return Maximum speed for the path segment.
	 */
	public double getMaxSpeed() {
		return maxSpeed;
	}

	/**
	 *
	 * @return Total distance from start of the segment to the end.
	 */
	public double getDistance() {
		return deltaDist;
	}

	/**
	 *
	 * @return X and Y offset from the start to the end of the segment.
	 */
	public Translation2d getDelta() {
		return delta;
	}
}