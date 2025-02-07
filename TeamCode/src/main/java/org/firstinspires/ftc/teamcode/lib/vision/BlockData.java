package org.firstinspires.ftc.teamcode.lib.vision;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class BlockData {
    public static final int COLOR_YELLOW = 0;
    public static final int COLOR_RED = 1;
    public static final int COLOR_BLUE = 2;

    short centerX;
    short centerY;
    short width;
    short height;
    float angle;
    int color;

    public float getAngle() {
        return angle;
    }

    public BlockData(ByteBuffer buffer) {
        buffer.order(ByteOrder.LITTLE_ENDIAN);
        this.centerX = buffer.getShort();
        this.centerY = buffer.getShort();
        this.width = buffer.getShort();
        this.height = buffer.getShort();
        this.angle = buffer.getFloat();
        this.color = buffer.getInt();
    }
}
