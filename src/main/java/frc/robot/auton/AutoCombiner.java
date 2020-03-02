package frc.robot.auton;

import frc.robot.actions.Action;
import frc.robot.actions.StartShootingAction;
import frc.robot.auton.AutoChooser.BeforeFirstShoot;
import frc.robot.auton.AutoChooser.BeforeSecondShoot;
import frc.robot.auton.AutoChooser.BeforeThirdShoot;
import frc.robot.pathGenerators.ActionsGenerator;
import frc.robot.subsystems.LukeIntake;
import frc.robot.subsystems.AustinTurret;
import frc.robot.subsystems.AustinTurret.TurretState;

public class AutoCombiner extends AutoOptionBase {
    private Action[] firstActions, secondActions, thirdActions;
    private BeforeFirstShoot beforeFirstShootSelected;

    public AutoCombiner(BeforeFirstShoot firstSelected, BeforeSecondShoot secondSelected,
                        BeforeThirdShoot thirdSelected) {
        beforeFirstShootSelected = firstSelected;
        firstActions = ActionsGenerator.getFirstSelectedActions(firstSelected);
        secondActions = ActionsGenerator.getSecondSelectedActions(secondSelected);
        thirdActions = ActionsGenerator.getThirdSelectedActions(thirdSelected);
    }

    @Override
    protected void routine() throws AutoEndEarlyException {
        //LukeIntake.getInstance().setHighSpeed();
        runAction(firstActions[0]);

        if(beforeFirstShootSelected == BeforeFirstShoot.TWO_ON_OPPO_TRENCH 
            || beforeFirstShootSelected == BeforeFirstShoot.HALF_STEAL 
            || beforeFirstShootSelected == BeforeFirstShoot.COMPLETE_STEAL) {
                runAction(firstActions[1]);
        }

        runAction(ActionsGenerator.shootingActions[0]);

        runAction(secondActions[0]);
        runAction(secondActions[1]);
        runAction(ActionsGenerator.shootingActions[1]);

        runAction(thirdActions[0]);
        //runAction(thirdActions[1]);
        runAction(ActionsGenerator.shootingActions[2]);

    }

}