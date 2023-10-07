// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class Util{
    public static double magnitude(Transform3d transform) {
        double x = transform.getX();
        double y = transform.getY();
        double z = transform.getZ();

        return Math.sqrt(x*x+y*y+z*z);

    }


}
