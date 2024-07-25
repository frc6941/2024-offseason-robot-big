package org.frcteam6941.pathplanning.astar;

import java.util.ArrayList;
import java.util.Collections;
import java.util.PriorityQueue;

import org.frcteam6941.pathplanning.astar.obstacles.Obstacle;
import org.frcteam6941.pathplanning.universal.Path;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class AStarPathPlanner {
    public PriorityQueue<Node> openSet = new PriorityQueue<>();
    public PriorityQueue<Node> closedSet = new PriorityQueue<>();

    /**
     * Plan the path.
     * 
     * @param startingPoint The start point
     * @param endingPoint   The end point
     * @param obstacles     The obstacles, can give null if there's none
     * @param stepx         The search step size in x direction
     * @param stepy         The search step size in y direction
     * @param hGain         The gain in heuristics value. The larger, the more A*
     *                      algorithm will behave like Best-fit Search with faster
     *                      executing speed sacrifising accuracy; the smaller, the
     *                      more A* algorithm will behave like Dijkstra Algorithm
     *                      and give more optimal results in a longer period
     * @param overtimeLimit The maximum executing time for the planning in seconds
     * @return ArrayList<Translation2d>} The path found. If not, will return null
     */
    public Path plan(Translation2d startingPoint, Translation2d endingPoint, Obstacle[] obstacles,
            double stepx, double stepy, double hGain, double overtimeLimit) {
        // Cleanup parameters
        stepx = Math.abs(stepx);
        stepy = Math.abs(stepy);

        // Clear open and closed set
        openSet = new PriorityQueue<>();
        closedSet = new PriorityQueue<>();

        // Add starting node to open set
        openSet.offer(new Node(startingPoint, endingPoint, 0, null));

        // Time the planning
        Timer timer = new Timer();
        timer.reset();
        timer.start();

        // Start of algorithm
        // Iterating if there's still nodes to be evaluted in the open set
        while (openSet.size() != 0 && timer.get() <= overtimeLimit) {
            // First, find the point with the minimal estimated score, add this node to
            // closed set, meaning that it is valuated
            Node minimalNode = openSet.poll();
            closedSet.offer(minimalNode);
            

            // Then, search the near positions
            searchNear(minimalNode, endingPoint, obstacles, stepx, stepy, hGain);
            searchNear(minimalNode, endingPoint, obstacles, stepx, -stepy, hGain);
            searchNear(minimalNode, endingPoint, obstacles, -stepx, stepy, hGain);
            searchNear(minimalNode, endingPoint, obstacles, -stepx, -stepy, hGain);
            searchNear(minimalNode, endingPoint, obstacles, stepx, 0, hGain);
            searchNear(minimalNode, endingPoint, obstacles, -stepx, 0, hGain);
            searchNear(minimalNode, endingPoint, obstacles, 0, stepy, hGain);
            searchNear(minimalNode, endingPoint, obstacles, 0, -stepy, hGain);

            // Check if the point that is close enough to the destination is added
            for (Node clNode : closedSet) {
                if (Math.abs(clNode.x - endingPoint.getX()) <= stepx
                        && Math.abs(clNode.y - endingPoint.getY()) <= stepy) {
                    ArrayList<Translation2d> resultPath = new ArrayList<>();
                    double pathLength = clNode.g + new Translation2d(clNode.x, clNode.y).getDistance(endingPoint);
                    resultPath.add(endingPoint);
                    // Use recursion to get original path, and return the trimmed one
                    System.out.println("Found route, Time: " + timer.get() + " seconds");
                    return new Path(pathLength, reconstructPath(clNode, resultPath));
                }
            }
        }
        System.out.println("Failed to found path.");
        
        // If all the points in open set is evaluted, then there's no possible path
        // found, return null
        return null;
    }

    private boolean checkObstacles(Node node, Obstacle[] obstacles) {
        if (obstacles == null) {
            return false;
        }
        for (Obstacle obstacle : obstacles) {
            if (obstacle.isInObstacle(node.x, node.y)) {
                return true;
            } else {

            }
        }
        return false;
    }

    public boolean checkLegal(Translation2d translation, Obstacle[] obstacles) {
        if (obstacles == null) {
            return true;
        }
        for (Obstacle obstacle : obstacles) {
            if (obstacle.isInObstacle(translation.getX(), translation.getY())) {
                return false;
            } else {

            }
        }
        return true;
    }

    private void searchNear(Node minimalNode, Translation2d destination, Obstacle[] obstacles, double stepx,
            double stepy, double hGain) {
        // End if the node is not in the first quadrant
        // Search a new node according to step
        Node node = new Node(new Translation2d(minimalNode.x + stepx, minimalNode.y + stepy), destination,
                minimalNode.g + Math.abs(stepx) + Math.abs(stepy), hGain, minimalNode);
        // If obstacles are encoutered, return
        if (checkObstacles(node, obstacles)) {
            return;
        } else {
            // If the node is already been evaluated in the closed set, return
            for (Node clNode : closedSet) {
                if (clNode.x == node.x && clNode.y == node.y) {
                    return;
                }
            }
            // If the node exist in open set, update its g value if the current node is a
            // shorter path
            for (Node opNode : openSet) {
                if (opNode.x == node.x && opNode.y == node.y) {
                    if (opNode.g > node.g) {
                        opNode.g = node.g;
                        opNode.previousNode = minimalNode;
                    } else {

                    }
                    return;
                }
            }
            // If the node does noe exist in open set, add it in the end
            openSet.offer(node);
        }
    }

    private ArrayList<Translation2d> reconstructPath(Node currentNode, ArrayList<Translation2d> currentTranslations) {
        currentTranslations.add(currentNode.geTranslation2d());
        if (currentNode.previousNode == null) {
            // Go from starting point to destination
            Collections.reverse(currentTranslations);
            return currentTranslations;
        } else {
            return reconstructPath(currentNode.previousNode, currentTranslations);
        }
    }

    private class Node implements Comparable<Node> {
        private double x;
        private double y;

        private double g = 0.0;
        private double h = 0.0;
        private Node previousNode = null;

        public Node(double x, double y, double g, double h, Node previousNode) {
            this.x = x;
            this.y = y;
            this.h = h;
            this.g = g;
            this.previousNode = previousNode;
        }

        public Node(Translation2d translation, Translation2d destination, double g, Node previousNode) {
            this(translation.getX(), translation.getY(), g, destination.minus(translation).getNorm(), previousNode);
        }

        public Node(Translation2d translation, Translation2d destination, double g, double hGain, Node previousNode) {
            this(translation.getX(), translation.getY(), g, destination.minus(translation).getNorm() * hGain,
                    previousNode);
        }

        public Translation2d geTranslation2d() {
            return new Translation2d(this.x, this.y);
        }

        @Override
        public int compareTo(Node other) {
            if (this.g + this.h > other.g + other.h) {
                return 1;
            } else if (this.g + this.h < other.g + other.h) {
                return -1;
            } else {
                return 0;
            }
        }

        @Override
        public String toString() {
            return String.format("Node(x:%.2f, y:%.2f)", x, y);
        }
    }
}
