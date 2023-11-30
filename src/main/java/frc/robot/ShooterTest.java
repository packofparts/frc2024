package frc.robot;

import java.util.*;

public class ShooterTest {
    // CONSTANTS
    int temporary_velocity = 20;
    double shooting_angle_in_degrees = 70.2;
    double change_height = 0.0;

    public double getTime() {
        double time = (0.0 - (20 * Math.sin(shooting_angle_in_degrees))) / -4.9;
        return time;
    }

    public double get_horizontal_distance() {
        double horizontal_distance =
                temporary_velocity * Math.cos(shooting_angle_in_degrees) * (getTime());
        return horizontal_distance;
    }
}
