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
   public static final Translation2d kGalactic_Search_ChallengeA = new Translation2d(30, 60);
   public static final Translation2d kGalactic_Search_ChallengeB = new Translation2d(30, 90);
   public static final Translation2d kSlalom = new Translation2d(60-29.5, 30);
   public static final Translation2d kBarrel = new Translation2d(60-29.5, 84);
   public static final Translation2d kBounce = new Translation2d(60-29.5, 90);

   //Fixed points
   public static final Translation2d kExampleStartingPoint = new Translation2d(20, 115);
   public static final Translation2d kBeforeAutoShootingPoint = new Translation2d(185, 40);
   public static final Translation2d kAutoShootingPoint = new Translation2d(205, 67);
   public static final Translation2d kPowerPortPosition = new Translation2d(0, 67);

   /** ------------------Galactic Search Challenge------ */
   public static final Translation2d path_A_Red1 = new Translation2d(90, 90);
   public static final Translation2d path_A_Red2 = new Translation2d(150, 60);
   public static final Translation2d path_A_Red3 = new Translation2d(180, 150);
   public static final Translation2d path_A_Red_end = new Translation2d(330, 150);

   public static final Translation2d path_A_Blue1 = new Translation2d(180, 30);
   public static final Translation2d path_A_Blue2 = new Translation2d(210, 120);
   public static final Translation2d path_A_Blue3 = new Translation2d(270, 90);
   public static final Translation2d path_A_Blue_end = new Translation2d(330, 90);

   public static final Translation2d path_B_Red1 = new Translation2d(90, 120);
   public static final Translation2d path_B_Red2 = new Translation2d(150, 60);
   public static final Translation2d path_B_Red3 = new Translation2d(210, 120);
   public static final Translation2d path_B_Red_end = new Translation2d(330, 120);

   public static final Translation2d path_B_Blue1 = new Translation2d(180, 60);
   public static final Translation2d path_B_Blue2 = new Translation2d(240, 120);
   public static final Translation2d path_B_Blue3 = new Translation2d(300, 60);
   public static final Translation2d path_B_Blue_end = new Translation2d(330, 60);

   /**-------------------Barrel Racing Path----------------- */
   public static final Translation2d conjunction = new Translation2d(225, 90);
   public static final Translation2d[] barrel_points = {
     new Translation2d(120, 90),
     new Translation2d(155, 80),
     new Translation2d(172, 50),
     new Translation2d(150, 35),
     new Translation2d(82, 50),
     new Translation2d(155, 80),
     new Translation2d(255, 90),
     new Translation2d(244, 124),
     new Translation2d(220, 130),
     new Translation2d(255, 90),
     new Translation2d(263, 50),
     new Translation2d(300, 35),
     new Translation2d(330, 60),
     new Translation2d(300, 85),
     new Translation2d(255, 90),
     new Translation2d(160, 100),
     new Translation2d(60-29.5, 105),
    };

   /**-------------------Slalom Path----------------- */
   public static final Translation2d[] slalom_points = {
     new Translation2d(90, 30),
     new Translation2d(120, 90),
     new Translation2d(180, 110),
     new Translation2d(250, 80),
     new Translation2d(270, 40),
     new Translation2d(300, 30),
     new Translation2d(315, 40),
     new Translation2d(335, 60),
     new Translation2d(315, 80),
     new Translation2d(300, 90),
     new Translation2d(285, 80),
     new Translation2d(270, 60),
     new Translation2d(255, 40),
     new Translation2d(255, 40),
     new Translation2d(180, 30),
     new Translation2d(150, 30),
     new Translation2d(100, 45),
     new Translation2d(75, 75),
     new Translation2d(60, 90)
    };

   /**-------------------Bounce Path----------------- */
   public static final Translation2d[] bounce_star1_1 = {new Translation2d(75, 100)};
   public static final Translation2d bounce_star1 = new Translation2d(90, 150);
   public static final Translation2d[] bounce_star2_1 = {
     new Translation2d(115, 80),
     new Translation2d(135, 45),
     new Translation2d(165, 45),
     new Translation2d(175, 55)
    };
   public static final Translation2d bounce_star2 = new Translation2d(180, 150);
   public static final Translation2d[] bounce_star3_1 = {
     new Translation2d(190, 45),
     new Translation2d(210, 35),
     new Translation2d(240, 35),
     new Translation2d(255, 40)
    };
   public static final Translation2d bounce_star3 = new Translation2d(270, 150);
   public static final Translation2d[] bounce_star4_1 = {new Translation2d(285, 120)};
   public static final Translation2d bounce_end = new Translation2d(330, 90);

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