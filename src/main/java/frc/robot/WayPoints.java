package frc.robot;

import frc.lib.math.Translation2d;

/**
 * this class contains all the way points 
 * used in autonomous paths
 */

 public class WayPoints {
   // Starting points
   public static final Translation2d kFaceOppoTrench = new Translation2d(120, -120);
   public static final Translation2d kFaceFrontBarrier = new Translation2d(120, 0);
   public static final Translation2d kFacePowerPort = new Translation2d(120, 67);

   //Fixed points
   public static final Translation2d kExampleStartingPoint = new Translation2d(20, 115);
   public static final Translation2d kBeforeAutoShootingPoint = new Translation2d(185, 40);
   public static final Translation2d kAutoShootingPoint = new Translation2d(205, 67);
   public static final Translation2d kPowerPortPosition = new Translation2d(0, 67);

   /*-----------TWO_ON_OPPO_TRENCH--------------*/
   public static final Translation2d to_oppo_cell_1 = new Translation2d(210, -120);
   public static final Translation2d to_oppo_cell_2 = new Translation2d(240, -120);
   public static final Translation2d to_oppo_cell_3 = new Translation2d(245, -135);

   //half steal
   public static final Translation2d to_oppo_trench_1 = new Translation2d(315, -140);
   public static final Translation2d to_oppo_trench_2 = new Translation2d(375, -140);
   //full steal
   public static final Translation2d to_oppo_trench_3 = new Translation2d(400, -125);
   public static final Translation2d to_oppo_trench_4 = new Translation2d(410, -100);
   public static final Translation2d to_oppo_trench_5 = new Translation2d(393, -57);

   //full steal reverse
   public static final Translation2d to_oppo_trench_1_reverse = to_oppo_trench_5;
   public static final Translation2d to_oppo_trench_2_reverse = new Translation2d(400, -75);
   public static final Translation2d to_oppo_trench_3_reverse = new Translation2d(380, -90);
   public static final Translation2d to_oppo_trench_4_reverse = new Translation2d(300, -125);
   public static final Translation2d to_oppo_trench_5_reverse = new Translation2d(240, -125);

   /* -------- Front Barrier------------ */
   public static final Translation2d to_front_barrier_1 = new Translation2d(180, -8);//the pivot
   public static final Translation2d to_front_barrier_2 = new Translation2d(215, 5);
   public static final Translation2d to_front_barrier_3 = to_front_barrier_1;
   public static final Translation2d to_front_barrier_4 = new Translation2d(210, -20);
   public static final Translation2d to_front_barrier_5 = new Translation2d(225, -14);

   /* ------ Back Barrier --------*/
   public static final Translation2d to_back_barrier_1 = new Translation2d(242, 80);//need turn
   public static final Translation2d to_back_barrier_2 = new Translation2d(250, 60);

   public static final Translation2d to_trench_pivot = new Translation2d(210, 135);//need turn

   /* ------Back Barrier To Front Barrier(BTF)----- */
   public static final Translation2d to_BTF_start = kAutoShootingPoint;
   //public static final Translation2d to_BTF_pivot_to_front_barrier = new Translation2d(185, 55);//need turn

   /* ------ Trench ------- */
   public static final Translation2d to_trench_1 = new Translation2d(210, 105);
   public static final Translation2d to_trench_1_add = new Translation2d(220, 125);
   public static final Translation2d to_trench_2 = new Translation2d(240, 135);
   public static final Translation2d to_trench_3 = new Translation2d(300, 135);

   //full trench
   public static final Translation2d to_trench_4 = new Translation2d(365, 140);
   //turn 180 if go to front barrier

 }