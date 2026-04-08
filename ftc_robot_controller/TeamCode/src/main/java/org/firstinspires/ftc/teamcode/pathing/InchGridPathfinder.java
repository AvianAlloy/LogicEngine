package org.firstinspires.ftc.teamcode.pathing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

/**
 * Lightweight A* pathfinder for FTC fields.
 *
 * Default setup is a 12ft x 12ft field with 1-inch resolution (144x144 cells).
 */
public final class InchGridPathfinder {
    public static final int DEFAULT_FIELD_INCHES = 144;

    public static final class Cell {
        public final int x;
        public final int y;

        public Cell(int x, int y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public String toString() {
            return "(" + x + "," + y + ")";
        }
    }

    private static final class Node implements Comparable<Node> {
        final int index;
        final float fScore;

        Node(int index, float fScore) {
            this.index = index;
            this.fScore = fScore;
        }

        @Override
        public int compareTo(Node other) {
            return Float.compare(this.fScore, other.fScore);
        }
    }

    private static final int[][] CARDINAL = new int[][]{
            {-1, 0}, {1, 0}, {0, -1}, {0, 1}
    };

    private static final int[][] DIAGONAL = new int[][]{
            {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    };

    private final int resolutionInches;
    private final int width;
    private final int height;
    private final boolean allowDiagonal;
    private final byte[] blocked;

    public InchGridPathfinder() {
        this(DEFAULT_FIELD_INCHES, DEFAULT_FIELD_INCHES, 1, true);
    }

    public InchGridPathfinder(int widthInches, int heightInches, int resolutionInches, boolean allowDiagonal) {
        if (resolutionInches <= 0) {
            throw new IllegalArgumentException("resolutionInches must be > 0");
        }

        this.resolutionInches = resolutionInches;
        this.width = widthInches / resolutionInches;
        this.height = heightInches / resolutionInches;

        if (width <= 0 || height <= 0) {
            throw new IllegalArgumentException("Field dimensions must result in at least 1 cell");
        }

        this.allowDiagonal = allowDiagonal;
        this.blocked = new byte[width * height];
    }

    public int getWidthCells() {
        return width;
    }

    public int getHeightCells() {
        return height;
    }

    public int getResolutionInches() {
        return resolutionInches;
    }

    public boolean inBounds(int x, int y) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    public void setBlocked(int x, int y, boolean value) {
        ensureInBounds(x, y);
        blocked[toIndex(x, y)] = (byte) (value ? 1 : 0);
    }

    public void setBlockedRect(int x0, int y0, int x1, int y1, boolean value) {
        int minX = Math.min(x0, x1);
        int maxX = Math.max(x0, x1);
        int minY = Math.min(y0, y1);
        int maxY = Math.max(y0, y1);

        ensureInBounds(minX, minY);
        ensureInBounds(maxX, maxY);

        byte cellValue = (byte) (value ? 1 : 0);
        for (int y = minY; y <= maxY; y++) {
            int rowOffset = y * width;
            for (int x = minX; x <= maxX; x++) {
                blocked[rowOffset + x] = cellValue;
            }
        }
    }

    public boolean isBlocked(int x, int y) {
        if (!inBounds(x, y)) {
            return true;
        }
        return blocked[toIndex(x, y)] != 0;
    }

    public List<Cell> findPath(Cell start, Cell goal) {
        ensureInBounds(start.x, start.y);
        ensureInBounds(goal.x, goal.y);

        if (isBlocked(start.x, start.y) || isBlocked(goal.x, goal.y)) {
            return Collections.emptyList();
        }

        int total = width * height;
        float[] gScore = new float[total];
        Arrays.fill(gScore, Float.POSITIVE_INFINITY);

        int[] parent = new int[total];
        Arrays.fill(parent, -1);

        byte[] closed = new byte[total];

        int startIndex = toIndex(start.x, start.y);
        int goalIndex = toIndex(goal.x, goal.y);

        PriorityQueue<Node> open = new PriorityQueue<>();
        gScore[startIndex] = 0f;
        open.add(new Node(startIndex, heuristic(start.x, start.y, goal.x, goal.y)));

        while (!open.isEmpty()) {
            Node current = open.poll();
            int cIndex = current.index;

            if (closed[cIndex] != 0) {
                continue;
            }

            if (cIndex == goalIndex) {
                return reconstruct(parent, goalIndex);
            }

            closed[cIndex] = 1;
            int cx = cIndex % width;
            int cy = cIndex / width;

            for (int[] d : CARDINAL) {
                visitNeighbor(cx, cy, d[0], d[1], 1f, goal, gScore, parent, closed, open);
            }

            if (allowDiagonal) {
                for (int[] d : DIAGONAL) {
                    visitNeighbor(cx, cy, d[0], d[1], 1.41421356f, goal, gScore, parent, closed, open);
                }
            }
        }

        return Collections.emptyList();
    }

    private void visitNeighbor(
            int cx,
            int cy,
            int dx,
            int dy,
            float moveCost,
            Cell goal,
            float[] gScore,
            int[] parent,
            byte[] closed,
            PriorityQueue<Node> open
    ) {
        int nx = cx + dx;
        int ny = cy + dy;

        if (!inBounds(nx, ny) || isBlocked(nx, ny)) {
            return;
        }

        // Prevent diagonal corner-cutting.
        if (dx != 0 && dy != 0 && isBlocked(cx + dx, cy) && isBlocked(cx, cy + dy)) {
            return;
        }

        int cIndex = toIndex(cx, cy);
        int nIndex = toIndex(nx, ny);
        if (closed[nIndex] != 0) {
            return;
        }

        float tentative = gScore[cIndex] + moveCost;
        if (tentative >= gScore[nIndex]) {
            return;
        }

        gScore[nIndex] = tentative;
        parent[nIndex] = cIndex;
        float fScore = tentative + heuristic(nx, ny, goal.x, goal.y);
        open.add(new Node(nIndex, fScore));
    }

    private List<Cell> reconstruct(int[] parent, int goalIndex) {
        ArrayList<Cell> reversed = new ArrayList<>();
        int cursor = goalIndex;
        while (cursor >= 0) {
            reversed.add(new Cell(cursor % width, cursor / width));
            cursor = parent[cursor];
        }
        Collections.reverse(reversed);
        return reversed;
    }

    private float heuristic(int x0, int y0, int x1, int y1) {
        int dx = Math.abs(x0 - x1);
        int dy = Math.abs(y0 - y1);

        if (!allowDiagonal) {
            return dx + dy;
        }

        int min = Math.min(dx, dy);
        int max = Math.max(dx, dy);
        return 1.41421356f * min + (max - min);
    }

    private int toIndex(int x, int y) {
        return y * width + x;
    }

    private void ensureInBounds(int x, int y) {
        if (!inBounds(x, y)) {
            throw new IllegalArgumentException("Cell out of bounds: (" + x + ", " + y + ")");
        }
    }

    public Cell inchesToCell(float xInches, float yInches) {
        int x = clamp((int) (xInches / resolutionInches), 0, width - 1);
        int y = clamp((int) (yInches / resolutionInches), 0, height - 1);
        return new Cell(x, y);
    }

    public Cell cellToInches(int x, int y) {
        ensureInBounds(x, y);
        return new Cell(x * resolutionInches, y * resolutionInches);
    }

    private int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}
