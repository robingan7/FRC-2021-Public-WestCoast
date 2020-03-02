package frc.lib.math;

/**
 * Stores a Translation2d and a Rotation
 */
public class RigidTransform implements Interpolable<RigidTransform> {

	public Rotation rotationMat;
	public Translation2d translationMat;

	public RigidTransform() {
		rotationMat = new Rotation();
		translationMat = new Translation2d();
	}

	public RigidTransform(Translation2d translation, Rotation rotation) {
		rotationMat = rotation;
		translationMat = translation;
	}

	@Override
	public RigidTransform interpolate(RigidTransform other, double percentage) {
		return new RigidTransform(this.translationMat.interpolate(other.translationMat, percentage),
				this.rotationMat.interpolate(other.rotationMat, percentage));
	}

	/**
	 * Translates delta rotated by our rotation matrix and rotates our rotation
	 * matrix by the other rotation matrix
	 *
	 * @param delta
	 *
	 * @return
	 */
	public RigidTransform transform(RigidTransform delta) {
		return new RigidTransform(translationMat.translateBy(delta.translationMat.rotateBy(rotationMat)),
				rotationMat.rotateBy(delta.rotationMat));
	}
}
