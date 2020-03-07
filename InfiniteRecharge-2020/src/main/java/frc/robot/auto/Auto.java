/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import frc.robot.Constants;

/**
 * Class that links together everything auto.
 */
public class Auto {
    /**
     * gets the approximate yaw ticks to look at the target
     * @param offsetY inches away from the side wall closest to the power port.
     * @return ticks to set the turret to
     */
    public static int getYawTicksToTarget(double offsetY) {
        double offsetYawTicks = (offsetY * Constants.TURRET_TARGET_TICKS_PER_INCH) + Constants.TURRET_APPROX_TARGET_TICKS_CLOSE;
        offsetYawTicks *= -1;
        return (int) offsetYawTicks;
    }
}
