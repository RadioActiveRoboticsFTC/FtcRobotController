package org.firstinspires.ftc.teamcode;

public class RobotLocation {
    // inches
    float x;
    float y;
    float z;
    float pitch;
    float roll;
    float heading;
    boolean valid;

    float distance(RobotLocation loc2) {
        float dx = loc2.x - x;
        float dy = loc2.y - y;
        return (float) Math.sqrt((dx*dx) + (dy*dy));
    }
}
