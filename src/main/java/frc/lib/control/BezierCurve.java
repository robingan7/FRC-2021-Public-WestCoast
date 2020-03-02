package frc.lib.control;

import java.util.ArrayList;
import java.util.Arrays;

import frc.lib.simple.JSONObject;
import frc.lib.math.Translation2d;

public class BezierCurve {

	public static class BezierPoint {

		private Translation2d start, prevTangent, nextTangent;
		private double speed;

		public BezierPoint(Translation2d prevTangent, Translation2d start, Translation2d nextTangent, double speed) {
			this.start = start;
			this.prevTangent = start.translateBy(prevTangent);
			this.nextTangent = start.translateBy(nextTangent);
			this.speed = speed;
		}
	}

	private ArrayList<BezierPoint> points;

	public BezierCurve(BezierPoint... bezierPoint) {
		this.points = new ArrayList<BezierPoint>();
		this.points.addAll(Arrays.asList(bezierPoint));
	}

	public void addPoints(BezierPoint... points) {
		this.points.addAll(Arrays.asList(points));
	}

	/**
	 * Returns a path computed from the list of BezierPoints in the curve
	 * 
	 * @param step
	 *            Determines how many points are generated. Step is > 0 and <
	 *            1.0. A smaller step generates more points.
	 * @return
	 */
	public Path computePath(double step) {
		if (points.size() > 1) {
			Path generatedPath = new Path(points.get(0).start);
			for (int i = 1; i < points.size(); i++) {
				BezierPoint firstPoint = points.get(i - 1);
				BezierPoint secondPoint = points.get(i);
				double startSpeed = firstPoint.speed;
				double endSpeed = secondPoint.speed;
				double diffSpeed = startSpeed - endSpeed;
				Segment firstSeg = new Segment(firstPoint.start, firstPoint.nextTangent, 0);
				Segment secondSeg = new Segment(firstPoint.nextTangent, secondPoint.prevTangent, 0);
				Segment thirdSeg = new Segment(secondPoint.prevTangent, secondPoint.start, 0);
				for (double j = step; j <= 1.0; j += step) {
					Translation2d A = firstSeg.getPointByPercentage(j);
					Translation2d B = secondSeg.getPointByPercentage(j);
					Translation2d C = thirdSeg.getPointByPercentage(j);
					Segment AB = new Segment(A, B, 0);
					Segment BC = new Segment(B, C, 0);
					Translation2d D = AB.getPointByPercentage(j);
					Translation2d E = BC.getPointByPercentage(j);
					Segment DE = new Segment(D, E, 0);
					Translation2d F = DE.getPointByPercentage(j);
					generatedPath.addPoint(F, (diffSpeed * j) + startSpeed);
				}
			}
			return generatedPath;
		} else {
			// Print error
			return null;
		}
	}

	public static BezierCurve parseJson(JSONObject json) {

		return new BezierCurve(null);
	}
}
