package frc.robot.subsystems;

import frc.robot.cycles.Subsystem_Cycle;
import frc.robot.Constants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends Subsystem_Cycle {
    private NetworkTable mNetworkTable;

    private Limelight() {
        mNetworkTable = NetworkTableInstance.getDefault().getTable(Constants.kLimelightTableName);
    }

    private static Limelight instance = new Limelight();
    public static Limelight getInstance() {
        if(instance == null) {
			instance = new Limelight();
		}
        return instance;
    }
    
    public static class VisionData {
        // INPUTS
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;
        public boolean isSeeTarget;
        public double distance;

        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    private VisionData visionData = new VisionData();
    private boolean mOutputsHaveChanged = true;

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != visionData.ledMode) {
            visionData.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void setPipeline(int mode) {
        if (mode != visionData.pipeline) {
            //RobotState.getInstance().resetVision();
            visionData.pipeline = mode;

            System.out.println(visionData.pipeline + ", " + mode);
            mOutputsHaveChanged = true;
        }
    }

    public synchronized boolean seesTarget() {
        return visionData.isSeeTarget;
    }

    public synchronized double xOffset() {
        return visionData.xOffset;
    } 

    public synchronized double yOffset() {
        return visionData.yOffset;
    }

    public synchronized double distance() {
        return visionData.distance;
    }

    public synchronized int ledMode() {
        return visionData.ledMode;
    }

    @Override
    public synchronized void update_subsystem() {
        visionData.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        visionData.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        visionData.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
        visionData.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        visionData.area = mNetworkTable.getEntry("ta").getDouble(0.0);
        visionData.isSeeTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

        visionData.distance = (Constants.kPowerPortHieght - Constants.kLimelightHeight) / Math.tan(Math.toRadians(Constants.kLimelightAngle + visionData.yOffset));
    }

    @Override
    public synchronized void move_subsystem() {
        if (visionData.givenLedMode != visionData.ledMode ||
                visionData.givenPipeline != visionData.pipeline) {
            System.out.println("Table has changed from expected, retrigger!!");
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {
            mNetworkTable.getEntry("ledMode").setNumber(visionData.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(visionData.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(visionData.pipeline);
            mNetworkTable.getEntry("stream").setNumber(visionData.stream);
            mNetworkTable.getEntry("snapshot").setNumber(visionData.snapshot);

            mOutputsHaveChanged = false;
        }
    }

    @Override
	public void sendDataToSmartDashboard() {
        SmartDashboard.putNumber("Target Distance", visionData.distance);
        //System.out.println("y: " + visionData.distance);
	}
}