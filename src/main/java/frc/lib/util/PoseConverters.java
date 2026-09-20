package frc.lib.util;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class PoseConverters {
    public static final Transform2d transform3dTo2d(Transform3d transform) {
        return new Transform2d(
                transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
    }
}
