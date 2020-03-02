package frc.robot.cycles;

public interface Cycle{
    void onStart(double time);
    void onLoop(double time);
    void onStop(double time);
}