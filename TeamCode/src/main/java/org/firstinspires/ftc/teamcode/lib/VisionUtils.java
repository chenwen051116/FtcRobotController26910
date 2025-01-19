package org.firstinspires.ftc.teamcode.lib;

public class VisionUtils {
    public static final int FRAME_WIDTH = 319;   // Maximum x coordinate
    public static final int FRAME_HEIGHT = 239;  // Maximum y coordinate
    public static final double BLOCK_WIDTH = 103;
    public static final double BLOCK_HEIGHT = 198;  // Make sure that block height is larger than width
    public static final double BLOCK_DIAGONAL_SQ = BLOCK_WIDTH * BLOCK_WIDTH + BLOCK_HEIGHT * BLOCK_HEIGHT;
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

    enum BlockStatus {
        UnableToCalculate,
        Horizontal,
        Vertical,
    }

    public static double EDGE_THRESHOLD = 2;  // The threshold for the frame to be considered "on the edge". Unit: # of pixels

    /**
     * Get the current status of the block given the parameters of the frame.
     */
    public static BlockStatus getStatus(double width, double height, double centerX, double centerY) {
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
        if(cnt >= 2) {
            // if the frame has more than 2 sides on the edge
            return BlockStatus.UnableToCalculate;
        }
        if (leftEdge || rightEdge) {
            return (calcAngle(height) >= PI_4) ? BlockStatus.Horizontal : BlockStatus.Vertical;
        } else {
            return (calcAngle(width) >= PI_4) ? BlockStatus.Vertical : BlockStatus.Horizontal;
        }
    }
}
