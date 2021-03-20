// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.enumeration;

/** 
 * Describes what set of constants are used by CyborgCommandAlignTurret
 */
public enum AlignModel {
    NORMAL(0, "Normal"),
    NEW_BALLS(1, "New Balls");

    private int index;
    private String name;

    AlignModel(int index, String name) {
        this.index = index;
        this.name = name;
    }

    public int getIndex() {
        return index;
    }

    public String getName() {
        return name;
    }
}