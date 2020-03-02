
package frc.lib.control;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Constants;
import frc.lib.math.Rotation;
import frc.lib.math.Translation2d;

/**
 * Contains a list of points and generated segments from these points. This can
 * be used to calculate lookahead in this class.
 */
public class Path {

	public static Path getEmptyPath() {
		return new Path(new Translation2d());
	}

	public static class DrivingData {
		public double remainingDist, maxSpeed;
		public Translation2d lookAheadPoint;
		public Translation2d closestPoint;
		public Translation2d currentSegEnd;
	}

	private List<Segment> segments;
	private Translation2d lastPoint;
	private Rotation endAngle = null;
	private double totalDist;
	private double finishedDist;
	private volatile boolean isEmpty;

	/**
	 * Contains a list of points and can create path segments with them.
	 * Lookahead is calculated with using this.
	 *
	 * @param start
	 *            Initial point in path.
	 */
	public Path(Translation2d start) {
		segments = new ArrayList<Segment>();
		lastPoint = start;
		isEmpty = true;
		totalDist = 0;
		finishedDist = 0;
	}

	/**
	 * Add another point to the path.
	 *
	 * @param x
	 * @param y
	 * @param speed
	 */
	public void addPoint(double x, double y, double speed) {
		Segment newSegment = new Segment(lastPoint.getX(), lastPoint.getY(), x, y, speed);
		segments.add(newSegment);
		totalDist += newSegment.getDistance();
		lastPoint = new Translation2d(x, y);
		isEmpty = false;
	}

	public void addPoint(Translation2d point, double speed) {
		addPoint(point.getX(), point.getY(), speed);
		isEmpty = false;
	}

	/**
	 * Sets the desired angle for the robot to end in. It does this by placing
	 * up to two points that have a 90 degree angle to allow the robot to
	 * converge on the path.
	 *
	 * @param angle
	 *            Angle for the robot to end in.
	 */
	public void setAngle(Rotation angle) {
		endAngle = angle;
	}

	public boolean isEmpty() {
		return isEmpty;
	}

	/**
	 * Used to alter path so that the ending angle will be in a certain
	 * direction. Obselete because it does not guarantee the ending angle
	 */
	@Deprecated
	public void processPoints() {
		if (endAngle != null) {
			Segment lastSegment = segments.get(segments.size() - 1);
			Rotation angleOfPath = lastSegment.getStart().getAngle(lastSegment.getEnd());
			Rotation rotatedEndAngle = angleOfPath.inverse().rotateBy(endAngle);
			boolean rotateLeft = rotatedEndAngle.sin() > 0;

			Translation2d finalSegmentStart = new Translation2d(Constants.MinimumTurningRadius, 0)
					.rotateBy(endAngle.flip());
			Translation2d secondSegmentStart = finalSegmentStart.rotateBy(Rotation.fromDegrees(rotateLeft ? 90 : -90));
			// The two points we potentially add
			finalSegmentStart = finalSegmentStart.translateBy(lastSegment.getEnd());
			secondSegmentStart = secondSegmentStart.translateBy(finalSegmentStart);

			segments.remove(segments.size() - 1);
			// Add both points if it is
			// sharper than a 90 degree turn
			if (rotatedEndAngle.cos() < 0) {
				segments.add(new Segment(lastSegment.getStart(), secondSegmentStart, lastSegment.getMaxSpeed()));
				segments.add(new Segment(secondSegmentStart, finalSegmentStart, lastSegment.getMaxSpeed()));
			} else {
				segments.add(new Segment(lastSegment.getStart(), finalSegmentStart, lastSegment.getMaxSpeed()));
			}
			segments.add(new Segment(finalSegmentStart, lastSegment.getEnd(), lastSegment.getMaxSpeed()));
		}
	}

	/**
	 * Prints all the points on the path.
	 */
	public synchronized void printAllPoints() {
		for (Segment segment : segments) {
			System.out.println(segment.getStart().getX() + "    " + segment.getStart().getY());
		}
		System.out.println(segments.get(segments.size() - 1).getEnd().getX() + "   "
				+ segments.get(segments.size() - 1).getEnd().getY());
	}

	/**
	 * TODO: Explain what's goin on in this function. Review with uncool
	 * kids(mentor)
	 *
	 * @param pose
	 *            Current robot position
	 * @param lookAheadDistance
	 *            Distance on the path to get the look ahead.
	 * @return
	 */
	public DrivingData getLookAheadPoint(Translation2d pose, double lookAheadDistance) {
		DrivingData data = new DrivingData();
		Translation2d closestPoint = segments.get(0).getClosestPoint(pose);
		Translation2d closestToRobot = closestPoint.inverse().translateBy(pose);
		// Remove old points that we have passed
		while (segments.size() > 1) {
			double distToClosest = Math.hypot(closestToRobot.getX(), closestToRobot.getY());
			Translation2d closestNextPoint = segments.get(1).getClosestPoint(pose);
			Translation2d closestNextToRobot = closestNextPoint.inverse().translateBy(pose);
			double distToNext = Math.hypot(closestNextToRobot.getX(), closestNextToRobot.getY());
			if (distToClosest > distToNext) {
				// Run commands that didn't run yet in segments being deleted
				finishedDist += segments.get(0).getDistance();
				segments.remove(0);
				closestPoint = closestNextPoint;
				closestToRobot = closestNextToRobot;
			} else {
				break;
			}
		}
		// Run commands when we zoom past
		double traveledDist = (segments.get(0).getPercentageOnSegment(pose) * segments.get(0).getDistance()
				+ finishedDist);
		double percentage = traveledDist / totalDist;
		
		data.closestPoint = closestPoint;
		data.currentSegEnd = segments.get(0).getEnd();
		Translation2d closestToEnd = closestPoint.inverse().translateBy(segments.get(0).getEnd());
		Translation2d closestToStart = segments.get(0).getStart().inverse().translateBy(closestPoint);

		lookAheadDistance += Math.hypot(closestToRobot.getX(), closestToRobot.getY());
		double remainingSegDist = Math.hypot(closestToEnd.getX(), closestToEnd.getY());
		data.remainingDist = remainingSegDist;
		data.maxSpeed = segments.get(0).getMaxSpeed();
		for (int i = 1; i < segments.size(); i++) {
			data.remainingDist += segments.get(i).getDistance();
		}
		// If the lookahead point lies on a path then return the point
		// else extrapolate past the end point from the last segment
		if (lookAheadDistance > remainingSegDist && segments.size() > 1) {
			lookAheadDistance -= remainingSegDist;
			for (int i = 1; i < segments.size(); i++) {
				if (lookAheadDistance > segments.get(i).getDistance() && i != (segments.size() - 1)) {
					lookAheadDistance -= segments.get(i).getDistance();
				} else {
					data.lookAheadPoint = segments.get(i).getPointByDistance(lookAheadDistance);
					break;
				}
			}
		} else {
			double distanceOnPath = segments.get(0).getDistance() - remainingSegDist + lookAheadDistance;
			data.lookAheadPoint = segments.get(0).getPointByDistance(distanceOnPath);
		}
		/*
		 * UDPServer.getInstance().send(data.lookAheadPoint.getX() + "," +
		 * data.lookAheadPoint.getY() + "," + pose.getX() + "," + pose.getY(),
		 * 5801);
		 */
		return data;
	}
}
