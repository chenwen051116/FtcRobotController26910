package org.firstinspires.ftc.teamcode.lib.vision;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class BlockData {
    public static final int COLOR_YELLOW = 1;
    public static final int COLOR_RED = 2;
    public static final int COLOR_BLUE = 4;

    double centerX;
    double centerY;
    double width;
    double height;
    double angle;
    int color;

    public double getAngle() {
        return angle;
    }

    public BlockData(double[] array, int startIndex) {
        this.centerX = array[startIndex];
        this.centerY = array[startIndex + 1];
        this.width = array[startIndex + 2];
        this.height = array[startIndex + 3];
        this.angle = array[startIndex + 4];
        this.color = (int) array[startIndex + 5];
    }
}
