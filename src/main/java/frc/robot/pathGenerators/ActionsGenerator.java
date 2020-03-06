package frc.robot.pathGenerators;

import frc.lib.control.Path;
import frc.lib.math.Translation2d;
import frc.robot.WayPoints;
import frc.robot.actions.Action;
import frc.robot.actions.DelayAction;
import frc.robot.actions.GetReadyToShootAction;
import frc.robot.actions.SeriesAction;
import frc.robot.actions.ParallelAction;
import frc.robot.actions.LambdaAction;
import frc.robot.actions.DriveAngleAction;
import frc.robot.actions.StartShootingAction;
import frc.robot.actions.DoNothingAction;
import frc.robot.actions.DrivePathAction;
import frc.robot.auton.AutoChooser.BeforeFirstShoot;
import frc.robot.auton.AutoChooser.BeforeSecondShoot;
import frc.robot.auton.AutoChooser.BeforeThirdShoot;
import frc.robot.subsystems.LukeIntake;
import frc.robot.auton.AutoChooser;
import frc.robot.Constants;

import java.util.Arrays;

public class ActionsGenerator {
    public static Action[] shootingActions = new Action[] {
        new StartShootingAction(Constants.kAutoShootingDuration), 
        new StartShootingAction(Constants.kAutoShootingDuration), 
        new StartShootingAction(Constants.kAutoShootingDuration)};

    private static int currentIndex = 0;

    public static Action[] getFirstSelectedActions(BeforeFirstShoot selected) {
        currentIndex = 0;
        resetActions();
        DrivePathAction drive_forward_path, drive_back_path;
        ParallelAction forwardAction = new ParallelAction(Arrays.asList(new DoNothingAction()));
        ParallelAction backAction = new ParallelAction(Arrays.asList(new DoNothingAction()));

        switch(selected) {
            case TWO_ON_OPPO_TRENCH:
                drive_forward_path = new DrivePathAction(PathGenerator.getTwoOppoTrench(Path.getEmptyPath(), false, true), 
                                    false);

                drive_back_path = new DrivePathAction(PathGenerator.getTwoOppoTrench(Path.getEmptyPath(), true, true), 
                                    true);

                forwardAction = new ParallelAction(Arrays.asList(
                    drive_forward_path,
                    new LambdaAction(() -> LukeIntake.getInstance().intakeInAuto())
                ));

                backAction = new ParallelAction(Arrays.asList(
                    drive_back_path,
                    new SeriesAction(Arrays.asList(new DelayAction(0.2), new GetReadyToShootAction()))
                ));
                break;

            case HALF_STEAL:
                drive_forward_path = new DrivePathAction(PathGenerator.getHalfSteal(Path.getEmptyPath(), false, true), 
                                    false);

                drive_back_path = new DrivePathAction(PathGenerator.getHalfSteal(Path.getEmptyPath(), true, true), 
                                    true);

                forwardAction = new ParallelAction(Arrays.asList(
                    drive_forward_path,
                    new LambdaAction(() -> LukeIntake.getInstance().intakeInAuto())
                ));

                backAction = new ParallelAction(Arrays.asList(
                    drive_back_path,
                    new SeriesAction(Arrays.asList(
                        new LambdaAction(() -> LukeIntake.getInstance().intakeInAuto()), 
                        new DelayAction(0.2),
                        new GetReadyToShootAction()))
                ));
                break;

            case COMPLETE_STEAL:
                drive_forward_path = new DrivePathAction(PathGenerator.getCompleteSteal(Path.getEmptyPath(), false), 
                                    false);

                drive_back_path = new DrivePathAction(PathGenerator.getCompleteSteal(Path.getEmptyPath(), true), 
                                    true);

                forwardAction = new ParallelAction(Arrays.asList(
                    drive_forward_path,
                    new LambdaAction(() -> LukeIntake.getInstance().intakeInAuto())
                ));

                backAction = new ParallelAction(Arrays.asList(
                    drive_back_path,
                    new SeriesAction(Arrays.asList(
                        new LambdaAction(() -> LukeIntake.getInstance().intakeInAuto()), 
                        new DelayAction(0.2),
                        new GetReadyToShootAction()))
                ));
                break;

            case FRONT_BARRIER:
                forwardAction = getFrontBarrierActions(AutoChooser.getInitialPoint(), true);
                break;

            case BACK_BARRIER:
                forwardAction = getBackBarrierActions(AutoChooser.getInitialPoint(), true, false, false);
                break;

            case DO_NOTHING:
                break;

            default:
                System.out.println("Invalid auto mode: " + selected);
                break;      
        }

        return new Action[]{forwardAction, backAction};
    }

    public static Action[] getSecondSelectedActions(BeforeSecondShoot selected) {
        currentIndex = 0;
        ParallelAction forwardAction = new ParallelAction(Arrays.asList(new DoNothingAction()));
        ParallelAction backAction = new ParallelAction(Arrays.asList(new DoNothingAction()));

        switch(selected) {
            case FRONT_BARRIER_TO_BACK_BARRIER:
                forwardAction = getFrontBarrierActions(WayPoints.kAutoShootingPoint, false);
                backAction = getBackBarrierActions(WayPoints.kAutoShootingPoint, true, true, false);
                break;

            case FRONT_BARRIER_TO_HALF_TRENCH:
                forwardAction = getFrontBarrierActions(WayPoints.kAutoShootingPoint, false);
                backAction = getTrenchActions(false, true, false);
                break;

            case BACK_BARRIER_TO_HALF_TRENCH:
                forwardAction = getBackBarrierActions(WayPoints.kAutoShootingPoint, false, true, true);
                backAction = getTrenchActions(true, false, false);
                break;
                
            case FULL_TRENCH:
                forwardAction = getTrenchActions(false, true, true);
                break;

            case BACK_BARRIER_TO_FULL_TRENCH:
                forwardAction = getBackBarrierActions(WayPoints.kAutoShootingPoint, false, true, true);
                backAction = getTrenchActions(true, false, true);
                break;

            case DO_NOTHING:
                break;

            default:
                System.out.println("Invalid auto mode: " + selected);
                break; 
        }

        return new Action[]{forwardAction, backAction};
    }

    public static Action[] getThirdSelectedActions(BeforeThirdShoot selected) {
        currentIndex = 1;
        ParallelAction forwardAction = new ParallelAction(Arrays.asList(new DoNothingAction()));
        ParallelAction backAction = new ParallelAction(Arrays.asList(new DoNothingAction()));

        switch(selected) {
            case FRONT_BARRIER:
                forwardAction = getFrontBarrierActions(WayPoints.kAutoShootingPoint, true);
                break;

            case BACK_BARRIER:
                forwardAction = getBackBarrierActions(WayPoints.kAutoShootingPoint, true, false, true);
                break;

            case TRENCH:
                forwardAction = getTrenchActions(false, false, false);
                break;

            case DO_NOTHING:
                break;

            default:
                System.out.println("Invalid auto mode: " + selected);
                break; 
        }

        return new Action[]{forwardAction, backAction};
    }


    private static ParallelAction getFrontBarrierActions(Translation2d startPoint, boolean shootAfter) {
        DrivePathAction drive_front_barrier = new DrivePathAction(PathGenerator.getFrontBarrierFirstTimePattern(new Path(startPoint), 
                                                        false), false);
        DrivePathAction drive_to_pivot = new DrivePathAction(PathGenerator.getFrontBarrierToPivot(Path.getEmptyPath()), true);
        DrivePathAction drive_to_barrier_again = new DrivePathAction(PathGenerator.getPivotToFrontBarrier(Path.getEmptyPath()), false);
        DrivePathAction drive_to_shooting = new DrivePathAction(PathGenerator.getFrontBarrierToShoot(Path.getEmptyPath()), true);

        if(!shootAfter) {
            return new ParallelAction(Arrays.asList(
                new SeriesAction(Arrays.asList(
                    drive_front_barrier, drive_to_pivot, drive_to_barrier_again, drive_to_shooting))
            ));
        }

        return new ParallelAction(Arrays.asList(
            new SeriesAction(Arrays.asList(
                drive_front_barrier, drive_to_pivot, drive_to_barrier_again, drive_to_shooting)), 
            new GetReadyToShootAction()  
        ));
    }
 
    private static ParallelAction getBackBarrierActions(Translation2d startPoint, boolean shootAfter, boolean needTurn, boolean isAfterShoot) {
        DriveAngleAction turn_to_back_barrier = new DriveAngleAction(WayPoints.to_back_barrier_1);
        DrivePathAction drive_to_back_barrier = new DrivePathAction(PathGenerator.getToBackBarrierPattern(new Path(startPoint), 
                                                    false, shootAfter), false);
        DrivePathAction drive_to_shooting_again = new DrivePathAction(PathGenerator.getToBackBarrierPattern(Path.getEmptyPath(), 
                                                    true, shootAfter), true);
        
        SeriesAction drives = new SeriesAction(Arrays.asList());
        SeriesAction shoots = new SeriesAction(Arrays.asList());

        if(needTurn) {
            if(isAfterShoot) {
                addTurnParallelToShooting(turn_to_back_barrier);
                drives = new SeriesAction(Arrays.asList(
                    drive_to_back_barrier, drive_to_shooting_again));
            } else {
                drives = new SeriesAction(Arrays.asList(
                    turn_to_back_barrier, drive_to_back_barrier, drive_to_shooting_again));
            }
            
        } else {
            drives = new SeriesAction(Arrays.asList(
                drive_to_back_barrier, drive_to_shooting_again));
        }

        if(shootAfter) {
            shoots = new SeriesAction(Arrays.asList(
                    new DelayAction(0.2),
                    new GetReadyToShootAction()));
        } else {
            shoots = new SeriesAction(Arrays.asList(
                new LambdaAction(() -> LukeIntake.getInstance().intakeInAuto())));
        }
        
        return new ParallelAction(Arrays.asList(drives, shoots));
    }

    private static ParallelAction getTrenchActions(boolean isFromBackBarrier, boolean isFromFrontBarrier, boolean isFull) {
        DrivePathAction drive_to_trench = new DrivePathAction(PathGenerator.getTrenchHalf(Path.getEmptyPath(), 
                                    false, isFromBackBarrier), 
                                    false);

        DrivePathAction drive_to_shoot = new DrivePathAction(PathGenerator.getTrenchHalf(Path.getEmptyPath(),
                            true, isFromBackBarrier), 
                            true);

        if(isFull) {
            drive_to_trench = new DrivePathAction(PathGenerator.getTrenchFull(Path.getEmptyPath(), 
                                    false, isFromBackBarrier), 
                                    false);

            drive_to_shoot = new DrivePathAction(PathGenerator.getTrenchFull(Path.getEmptyPath(),
                                true, isFromBackBarrier), 
                                true);
        }

        SeriesAction drives = new SeriesAction(Arrays.asList(drive_to_trench, drive_to_shoot));

        if(isFromBackBarrier) {
            drives = new SeriesAction(Arrays.asList(
                new DriveAngleAction(WayPoints.to_trench_2), 
                drive_to_trench, 
                drive_to_shoot));
        } else if(isFromFrontBarrier) {
            addTurnParallelToShooting(new DriveAngleAction(WayPoints.to_trench_1));
            drives = new SeriesAction(Arrays.asList(drive_to_trench, drive_to_shoot));
        }

        return new ParallelAction(Arrays.asList(
            drives,
            new SeriesAction(Arrays.asList(
                new DelayAction(0.2),
                new GetReadyToShootAction()))
        ));
    }

    private static void addTurnParallelToShooting(DriveAngleAction addAction) {
        shootingActions[currentIndex] = new ParallelAction(Arrays.asList(
            addAction,
            new StartShootingAction(Constants.kAutoShootingDuration)
        ));
    }

    private static void resetActions() {
        shootingActions = new Action[] {
            new StartShootingAction(Constants.kAutoShootingDuration), 
            new StartShootingAction(Constants.kAutoShootingDuration), 
            new StartShootingAction(Constants.kAutoShootingDuration)};
    }
}