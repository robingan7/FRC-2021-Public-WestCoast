package frc.robot.pathGenerators;

import frc.robot.WayPoints;
import frc.lib.control.Path;
import frc.lib.math.Translation2d;
import frc.robot.auton.AutoChooser;

public class PathGenerator {

    public static Path getTwoOppoTrench(Path pathIn, boolean isBack, boolean isEmptyPath) {
        if(!isBack) {
            pathIn = new Path(AutoChooser.getInitialPoint());
            pathIn.addPoint(WayPoints.to_oppo_cell_1, 90);
            pathIn.addPoint(WayPoints.to_oppo_cell_2, 70);
            pathIn.addPoint(WayPoints.to_oppo_cell_3, 70);
        } else {
            if(isEmptyPath) {
                pathIn = new Path(WayPoints.to_oppo_cell_3);
            } else {
                pathIn.addPoint(WayPoints.to_oppo_cell_3, 90);
            }
            pathIn.addPoint(WayPoints.kBeforeAutoShootingPoint, 120);
            pathIn.addPoint(WayPoints.kAutoShootingPoint, 120);
        }
        
        return pathIn;
    }

    public static Path getHalfSteal(Path pathIn, boolean isBack, boolean isEmptyPath) {
        if(!isBack) {
            pathIn = getTwoOppoTrench(pathIn, false, true);
            pathIn.addPoint(WayPoints.to_oppo_trench_1, 60);
            pathIn.addPoint(WayPoints.to_oppo_trench_2, 60);
        } else {
            if(isEmptyPath) {
                pathIn = new Path(WayPoints.to_oppo_trench_2);
            } else {
                pathIn.addPoint(WayPoints.to_oppo_trench_2, 60);
            }
            pathIn.addPoint(WayPoints.to_oppo_trench_1, 60);
            pathIn = getTwoOppoTrench(pathIn, true, false);
        }

        return pathIn;
    }

    public static Path getCompleteSteal(Path pathIn, boolean isBack) {
        if(!isBack) {
            pathIn = getHalfSteal(pathIn, false, true);
            pathIn.addPoint(WayPoints.to_oppo_trench_3, 60);
            pathIn.addPoint(WayPoints.to_oppo_trench_4, 60);
            pathIn.addPoint(WayPoints.to_oppo_trench_5, 60);
        } else {
            pathIn = new Path(WayPoints.to_oppo_trench_1_reverse);
            pathIn.addPoint(WayPoints.to_oppo_trench_2_reverse, 60);
            pathIn.addPoint(WayPoints.to_oppo_trench_3_reverse, 60);
            pathIn.addPoint(WayPoints.to_oppo_trench_4_reverse, 60);
            pathIn.addPoint(WayPoints.to_oppo_trench_5_reverse, 60);
            pathIn.addPoint(WayPoints.kBeforeAutoShootingPoint, 60);
            pathIn.addPoint(WayPoints.kAutoShootingPoint, 60);
        }

        return pathIn;
    }

    public static Path getFrontBarrierFirstTimePattern(Path pathIn, boolean isBack) {
        if(!isBack) {
            if(pathIn == null) {
                return pathIn;
            }
            pathIn.addPoint(WayPoints.to_front_barrier_1, 60);
            pathIn.addPoint(WayPoints.to_front_barrier_2, 60);
        } else {
            pathIn = new Path(WayPoints.to_front_barrier_2);
            pathIn.addPoint(WayPoints.to_front_barrier_1, 90);
            pathIn.addPoint(WayPoints.kAutoShootingPoint, 90);
        }

        return pathIn;
    }

    public static Path getFrontBarrierToPivot(Path pathIn) {
        pathIn = new Path(WayPoints.to_front_barrier_2);
        pathIn.addPoint(WayPoints.to_front_barrier_1, 120);
        return pathIn;
    }

    public static Path getPivotToFrontBarrier(Path pathIn) {
        pathIn = new Path(WayPoints.to_front_barrier_1);
        pathIn.addPoint(WayPoints.to_front_barrier_4, 120);
        pathIn.addPoint(WayPoints.to_front_barrier_5, 120);
        return pathIn;
    }

    public static Path getFrontBarrierToShoot(Path pathIn) {
        pathIn = new Path(WayPoints.to_front_barrier_5);
        pathIn.addPoint(WayPoints.to_front_barrier_4, 120);
        pathIn.addPoint(WayPoints.kBeforeAutoShootingPoint, 120);
        pathIn.addPoint(WayPoints.kAutoShootingPoint, 120);
        return pathIn;
    }

    public static Path getToBackBarrierPattern(Path pathIn, boolean isBack, boolean shootAfter) {
        if(!isBack) {
            if(pathIn == null) {
                return pathIn;
            }
            pathIn.addPoint(WayPoints.to_back_barrier_1, 90);
            pathIn.addPoint(WayPoints.to_back_barrier_2, 90);
        } else {
            pathIn = new Path(WayPoints.to_back_barrier_2);
            pathIn.addPoint(WayPoints.to_back_barrier_1, 90);

            if(shootAfter) {
                pathIn.addPoint(WayPoints.kAutoShootingPoint, 120);
            } else {
                pathIn.addPoint(WayPoints.to_trench_pivot, 120);
            }
        }
        
        return pathIn;
    }

    public static Path getTrenchHalf(Path pathIn, boolean isBack, boolean isFromBackBarrier) {
        if(!isBack) {
            pathIn = new Path(WayPoints.kAutoShootingPoint);

            if(!isFromBackBarrier){
                pathIn.addPoint(WayPoints.to_trench_1, 60);
            }
            pathIn.addPoint(WayPoints.to_trench_1_add, 90);
            pathIn.addPoint(WayPoints.to_trench_2, 100);
            pathIn.addPoint(WayPoints.to_trench_3, 100);
        } else {
            pathIn = new Path(WayPoints.to_trench_3);
            pathIn.addPoint(WayPoints.kAutoShootingPoint, 100);
        }

        return pathIn;
    }

    public static Path getTrenchFull(Path pathIn, boolean isBack, boolean isFromBackBarrier) {
        if(!isBack) {
            pathIn = getTrenchHalf(pathIn, false, isFromBackBarrier);
            pathIn.addPoint(WayPoints.to_trench_4, 90);
        } else {
            pathIn = new Path(WayPoints.to_trench_4);
            pathIn.addPoint(WayPoints.kAutoShootingPoint, 90);
        }

        return pathIn;
    }

    public static Path getPathA_R() {
        Path path = new Path(WayPoints.kGalactic_Search_ChallengeA);
        path.addPoint(WayPoints.path_A_Red1, 100);
        path.addPoint(WayPoints.path_A_Red2, 100);
        path.addPoint(WayPoints.path_A_Red3, 100);
        path.addPoint(WayPoints.path_A_Red_end, 100);
        return path;
    }

    public static Path getPathA_B() {
        Path path = new Path(WayPoints.kGalactic_Search_ChallengeA);
        path.addPoint(WayPoints.path_A_Blue1, 100);
        path.addPoint(WayPoints.path_A_Blue2, 100);
        path.addPoint(WayPoints.path_A_Blue3, 100);
        path.addPoint(WayPoints.path_A_Blue_end, 100);
        return path;
    }

    public static Path getPathB_R() {
        Path path = new Path(WayPoints.kGalactic_Search_ChallengeB);
        path.addPoint(WayPoints.path_B_Red1, 100);
        path.addPoint(WayPoints.path_B_Red2, 100);
        path.addPoint(WayPoints.path_B_Red3, 100);
        path.addPoint(WayPoints.path_B_Red_end, 100);
        return path;
    }

    public static Path getPathB_B() {
        Path path = new Path(WayPoints.kGalactic_Search_ChallengeB);
        path.addPoint(WayPoints.path_B_Blue1, 100);
        path.addPoint(WayPoints.path_B_Blue2, 100);
        path.addPoint(WayPoints.path_B_Blue3, 100);
        path.addPoint(WayPoints.path_B_Blue_end, 100);
        return path;
    }

    public static Path getBarrel() {
        Path path = new Path(WayPoints.kBarrel);

        for(Translation2d point : WayPoints.barrel_points) {
            path.addPoint(point, 50);
        }
        return path;
    }

    public static Path getSlatom() {
        Path path = new Path(WayPoints.kSlalom);

        for(Translation2d point : WayPoints.slalom_points) {
            path.addPoint(point, 60);
        }
        return path;
    }

    public static Path getBounce_toStar1() {
        Path path = new Path(WayPoints.kBounce);
        for(Translation2d point : WayPoints.bounce_star1_1) {
            path.addPoint(point, 100);
        }
        path.addPoint(WayPoints.bounce_star1, 100);
        return path;
    }

    public static Path getBounce_toStar2() {
        Path path = new Path(WayPoints.bounce_star1);
        for(Translation2d point : WayPoints.bounce_star2_1) {
            path.addPoint(point, 100);
        }
        path.addPoint(WayPoints.bounce_star2, 100);

        return path;
    }

    public static Path getBounce_toStar3() {
        Path path = new Path(WayPoints.bounce_star2);
        for(Translation2d point : WayPoints.bounce_star3_1) {
            path.addPoint(point, 100);
        }
        path.addPoint(WayPoints.bounce_star3, 100);

        return path;
    }

    public static Path getBounce_toEnd() {
        Path path = new Path(WayPoints.bounce_star3);
        for(Translation2d point : WayPoints.bounce_star4_1) {
            path.addPoint(point, 100);
        }
        path.addPoint(WayPoints.bounce_end, 100);

        return path;
    }

}
