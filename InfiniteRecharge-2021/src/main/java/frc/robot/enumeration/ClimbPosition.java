/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.enumeration;

/**
 * Add your docs here.
 */
public enum ClimbPosition {
        LOWEST(1, "Lowest Position"),
        ON_WHEEL(2, "On Control Wheel"),
        ABOVE_WHEEL(3, "Above Control Wheel"),
        HIGHEST(4, "Highest Position");
    
        private final int value;
        private final String name;
    
        ClimbPosition(int value, String name) {
            this.value = value;
            this.name = name;
        }
    
        public int toInt() {
            return value;
        }
    
        public String toString() {
            return name;
        }
}
