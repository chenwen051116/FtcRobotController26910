package org.firstinspires.ftc.teamcode.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class VisionUtils {
    public static final int FRAME_WIDTH = 319;   // Maximum x coordinate
    public static final int FRAME_HEIGHT = 239;  // Maximum y coordinate
    public static final double BLOCK_WIDTH = 83;
    public static final double BLOCK_HEIGHT = 186;  // Make sure that block height is larger than width
    public static final double BLOCK_DIAGONAL_SQ = BLOCK_WIDTH * BLOCK_WIDTH + BLOCK_HEIGHT * BLOCK_HEIGHT;
    public static final double BLOCK_DIAGONAL = Math.sqrt(BLOCK_DIAGONAL_SQ);
    public static final double BLOCK_DIAGONAL_ANGLE = Math.atan(BLOCK_WIDTH / BLOCK_HEIGHT);

    /**
     * Calculates the angle between the long edge of the block and the horizontal line.
     * Assume that at least 3 of the 4 vertices of the block is visible, and the block's full width
     * is captured.
     *      *
     * +---------------+
     * *               |
     * |               |
     * |               *
     * |               |
     * +----------*----+
     * @param frameWidth Width of the frame
     * @return Angle
     */
    public static double calcAngle(double frameWidth) {
        return BLOCK_DIAGONAL_ANGLE + Math.atan(Math.sqrt(BLOCK_DIAGONAL_SQ - frameWidth * frameWidth) / frameWidth);
    }

    public static Pose2d ERROR_VALUE = new Pose2d(Double.NaN, Double.NaN, Double.NaN);
    public static double EDGE_THRESHOLD = 2;  // The threshold for the frame to be considered "on the edge". Unit: # of pixels

    /**
     * Get the current status of the block given the parameters of the frame.
     * @return A Pose2d structure giving the block center's X and Y position, and the block's angle
     * (0°=horizontal, 90°=vertical).
     * If the return value is (NaN, NaN, NaN), that means the program fails to find the parameters
     * of the block
     */
    public static Pose2d getStatus(double width, double height, double centerX, double centerY) {
        if(width < BLOCK_WIDTH+10 || height < BLOCK_WIDTH+10) {
            return ERROR_VALUE;
        }
        double w_2 = width / 2;
        double h_2 = height / 2;
        double x1 = centerX - w_2;
        double x2 = centerX + w_2;
        double y1 = centerY - h_2;
        double y2 = centerY + h_2;
        // check if the frame is on the edge
        boolean leftEdge = (x1 < EDGE_THRESHOLD);
        boolean rightEdge = (x2 >= FRAME_WIDTH - EDGE_THRESHOLD);
        boolean upEdge = (y1 < EDGE_THRESHOLD);
        boolean downEdge = (y2 >= FRAME_HEIGHT - EDGE_THRESHOLD);
        int cnt = (leftEdge?1:0) + (rightEdge?1:0) + (upEdge?1:0) + (downEdge?1:0);

        double PI_4 = Math.PI / 4;  // 45 degree
        double PI_2 = Math.PI / 2;  // 90 degree
        if(cnt >= 2) {
            // if the frame has more than 2 sides on the edge
            return ERROR_VALUE;
        }
        if (leftEdge || rightEdge) {
            double angle = calcAngle(height);
            if(Double.isNaN(angle)) {
                return ERROR_VALUE;
            }
            double length = 0.5 * BLOCK_DIAGONAL * Math.sin(angle + BLOCK_DIAGONAL_ANGLE);
            double blockCenterX = leftEdge ? (centerX + width / 2 - length) : (centerX - width / 2 + length);
            return new Pose2d(blockCenterX, centerY, PI_2 - angle);
        } else {
            double angle = calcAngle(width);
            if(Double.isNaN(angle)) {
                return ERROR_VALUE;
            }
            double length = 0.5 * BLOCK_DIAGONAL * Math.sin(angle + BLOCK_DIAGONAL_ANGLE);
            double blockCenterY = upEdge ? (centerY + width / 2 - length) : (centerY - width / 2 + length);
            return new Pose2d(centerX, blockCenterY, angle);
        }
    }

    /**
     * Get the value to set to the servo given an arrow on the screen, telling the block's direction.
     * @param x1 first x coordinate of the arrow.
     * @param y1 first y coordinate of the arrow.
     * @param x2 second x coordinate of the arrow.
     * @param y2 second y coordinate of the arrow.
     * @return A number between 0 and 1. 0 is pointing to the left, 0.5 is pointing straight up/down,
     *         and 1 is pointing to the right.
     */
    public static double getServoValFromArrow(double x1, double y1, double x2, double y2) {
        double length = Math.hypot(x2 - x1, y2 - y1);
        double angle = Math.atan((y2 - y1) / (x2 - x1));
        if(length < BLOCK_WIDTH) {
            angle += Math.PI / 2;
        }
        if(angle < 0) {
            angle += Math.PI;
        } else if(angle > Math.PI) {
            angle -= Math.PI;
        }
        return (Math.PI - angle) / Math.PI;
    }
}
