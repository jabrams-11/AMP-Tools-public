#include "MyBugAlgorithm.h"
#include <iostream> // For error messages

// Define a global (file-static) step size
static constexpr double GLOBAL_STEP_SIZE = 0.01;

// Helper function to check if a point is inside any obstacle and return which obstacle
int MyBugAlgorithm::isInCollision(const Eigen::Vector2d& point, const amp::Problem2D& problem) {
    // Check if point is outside workspace bounds
    if (point.x() < problem.x_min || point.x() > problem.x_max ||
        point.y() < problem.y_min || point.y() > problem.y_max) {
        return -1; // Outside bounds
    }
    
    // Check each obstacle and return the index of the one we hit
    for (int obs_idx = 0; obs_idx < problem.obstacles.size(); obs_idx++) {
        const auto& obstacle = problem.obstacles[obs_idx];
        const auto& vertices = obstacle.verticesCCW();
        int n = vertices.size();
        bool inside = false;
        
        // Ray casting algorithm to check if point is inside polygon
        for (int i = 0, j = n - 1; i < n; j = i++) {
            if (((vertices[i].y() > point.y()) != (vertices[j].y() > point.y())) &&
                (point.x() < (vertices[j].x() - vertices[i].x()) * (point.y() - vertices[i].y()) / 
                 (vertices[j].y() - vertices[i].y()) + vertices[i].x())) {
                inside = !inside;
            }
        }
        
        if (inside) {
            return obs_idx; // Return the obstacle index we hit
            LOG("Hit obstacle index: " << obs_idx);
        }
    }
    return -1; // No collision
}

// Helper function to rotate a vector 90 degrees clockwise
Eigen::Vector2d MyBugAlgorithm::rotateClockwise90(const Eigen::Vector2d& vec) {
    return Eigen::Vector2d(vec.y(), -vec.x());
}

// Helper function to rotate a vector 90 degrees counter-clockwise
Eigen::Vector2d MyBugAlgorithm::rotateCounterClockwise90(const Eigen::Vector2d& vec) {
    return Eigen::Vector2d(-vec.y(), vec.x());
}

// Combined function to find closest boundary point AND wall normal from specific obstacle
std::pair<Eigen::Vector2d, Eigen::Vector2d> MyBugAlgorithm::findClosestBoundaryAndNormal(const Eigen::Vector2d& point, const amp::Problem2D& problem, int obstacle_index) {
    double min_distance = std::numeric_limits<double>::max();
    Eigen::Vector2d closest_point = point;
    Eigen::Vector2d wall_normal = Eigen::Vector2d(1, 0); // Default normal
    
    // Only check the specified obstacle
    if (obstacle_index >= 0 && obstacle_index < problem.obstacles.size()) {
        const auto& obstacle = problem.obstacles[obstacle_index];
        const auto& vertices = obstacle.verticesCCW();
        int n = vertices.size();
        
        for (int i = 0; i < n; i++) {
            Eigen::Vector2d v1 = vertices[i];
            Eigen::Vector2d v2 = vertices[(i + 1) % n];
            
            // Find closest point on this edge
            Eigen::Vector2d edge = v2 - v1;
            double edge_length = edge.norm();
            
            if (edge_length < 1e-10) continue; // Skip zero-length edges
            
            double t = std::max(0.0, std::min(1.0, (point - v1).dot(edge) / (edge_length * edge_length)));
            Eigen::Vector2d closest_on_edge = v1 + t * edge;
            
            double distance = (point - closest_on_edge).norm();
            if (distance < min_distance) {
                min_distance = distance;
                closest_point = closest_on_edge;
                
                // Print the edge that was chosen
                std::cout << "Chosen edge from obstacle " << obstacle_index << ": v1=(" << v1.x() << ", " << v1.y() << ") to v2=(" << v2.x() << ", " << v2.y() << "), distance=" << distance << std::endl;
                
                // Calculate outward normal for CCW polygon
                // For edge v1->v2, outward normal is perpendicular to the RIGHT
                // Since polygons are CCW, outward normal points away from obstacle
                Eigen::Vector2d edge_unit = edge / edge_length;
                wall_normal = Eigen::Vector2d(-edge_unit.y(), edge_unit.x()); // Right perpendicular = outward normal
            }
        }
    }
    
    // Verify normal points outward by checking if it points away from obstacle center
    if (obstacle_index >= 0 && obstacle_index < problem.obstacles.size()) {
        // Find center of the specific obstacle
        Eigen::Vector2d obstacle_center = Eigen::Vector2d::Zero();
        int vertex_count = 0;
        
        for (const auto& vertex : problem.obstacles[obstacle_index].verticesCCW()) {
            obstacle_center += vertex;
            vertex_count++;
        }
        
        if (vertex_count > 0) {
            obstacle_center /= vertex_count;
            
            // Check if normal points away from obstacle center
            Eigen::Vector2d to_obstacle = obstacle_center - closest_point;
            if (wall_normal.dot(to_obstacle) > 0) {
                // Normal points toward obstacle, flip it
                wall_normal = -wall_normal;
            }
        }
    }
    
    return std::make_pair(closest_point, wall_normal.normalized());
}

// The fully implemented Bug 1 algorithm
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    Eigen::Vector2d current_pos = problem.q_init;
    path.waypoints.push_back(current_pos);

    int max_steps = 10000; // Prevent infinite loops REMOVE LATER
    int step_count = 0;

    // Main loop: Continue until the goal is reached
    while ((current_pos - problem.q_goal).norm() > GLOBAL_STEP_SIZE && step_count < max_steps) {
        step_count++;
        
        // === STATE 1: MOVE TOWARDS GOAL ===
        Eigen::Vector2d goal_direction = (problem.q_goal - current_pos).normalized();

        Eigen::Vector2d next_pos = current_pos + GLOBAL_STEP_SIZE * goal_direction;

        // Check for collision
        int collision_obstacle = isInCollision(next_pos, problem);
        if (collision_obstacle == -1) {
            // No collision, take the step
            current_pos = next_pos;
            path.waypoints.push_back(current_pos);
        } else {
            // Collision detected! Enter wall-following mode.
            
            // === STATE 2: CIRCUMNAVIGATE OBSTACLE (Right-Hand Rule) === amend infuture follow counterclockwise iterate thru points on the obstacle??? keep checkign forward vector for collisions with other obstacles
            Eigen::Vector2d hit_point = current_pos; // this might need to be changed to the point on the obstacle boundary that we hit
            Eigen::Vector2d closest_point = hit_point;
            Eigen::Vector2d circumnavigation_start = hit_point; // Save starting position for circumnavigation
            double min_dist_to_goal = (hit_point - problem.q_goal).norm();

            // Use the obstacle we just hit
            int current_obstacle = collision_obstacle;
            LOG("Hit obstacle index: " << current_obstacle);

            // Find the closest point on the obstacle boundary AND get wall normal
            auto [boundary_point, wall_normal] = findClosestBoundaryAndNormal(current_pos, problem, current_obstacle);
            
            // For clockwise wall following, we want to move perpendicular to the normal
            // (tangent direction). For CCW polygons, outward normal points away from obstacle
            // So tangent = rotate normal 90 degrees clockwise
            Eigen::Vector2d wall_follow_dir = rotateClockwise90(wall_normal);
            
            LOG("Hit obstacle at (" << current_pos.x() << "," << current_pos.y() << ")");
            LOG("Boundary point (" << boundary_point.x() << "," << boundary_point.y() << ")");
            LOG("Wall normal (" << wall_normal.x() << "," << wall_normal.y() << ")");
            LOG("Wall follow dir (" << wall_follow_dir.x() << "," << wall_follow_dir.y() << ")");
            // Pause here for debugging
           // std::cout << "Press Enter to continue..." << std::endl;
            //std::cin.get();

            // Follow the wall until we can leave
            int wall_follow_steps = 0; //  REMOVE LATER
            while (wall_follow_steps < 20000) { // Safety limit for wall following
                wall_follow_steps++;
                
                // Check if wall opens up to the right - if so, turn right to keep following wall
                LOG("Wall follow dir (" << wall_follow_dir.x() << "," << wall_follow_dir.y() << ")");
                LOG("Robot pos (" << current_pos.x() << "," << current_pos.y() << ")");
                Eigen::Vector2d right_dir = rotateClockwise90(wall_follow_dir);
                Eigen::Vector2d right_pos = current_pos + GLOBAL_STEP_SIZE* 1 * right_dir;

                LOG("Right pos (" << right_pos.x() << "," << right_pos.y() << ")");
             //   std::cout << "Press Enter to continue..." << std::endl;
              //  std::cin.get();
                
                // Check if we can turn right (wall opens up to the right)
                int right_collision = isInCollision(right_pos, problem);
                if (right_collision == -1) {
                  
                    // Wall opens up to the right - turn right to keep following wall
                    // First take a step in the current wall follow direction
                    // Eigen::Vector2d step_pos = current_pos + GLOBAL_STEP_SIZE * wall_follow_dir;
                    // if (!isInCollision(step_pos, problem)) {
                    //     current_pos = step_pos;
                    //     path.waypoints.push_back(current_pos);
                    // }
                    // Then turn right
                    current_pos = current_pos + GLOBAL_STEP_SIZE * right_dir;
                    wall_follow_dir = right_dir;
                    path.waypoints.push_back(current_pos);
                    LOG("Wall follow - TURNING RIGHT (wall opens up) at (" << current_pos.x() << "," << current_pos.y() << ")");
                } else {
                    // Try to move along the tangent (wall following direction)
                    Eigen::Vector2d next_pos = current_pos + GLOBAL_STEP_SIZE * wall_follow_dir;
                    LOG("Next pos (" << next_pos.x() << "," << next_pos.y() << ")");
                    int next_collision = isInCollision(next_pos, problem);
                    LOG("Next collision (" << next_collision << ")");
                    LOG("Current obstacle (" << current_obstacle << ")");
                    if (next_collision == -1) {
                        // Can move along tangent
                        current_pos = next_pos;
                        path.waypoints.push_back(current_pos);
                       // LOG("Wall follow - MOVING ALONG WALL at (" << current_pos.x() << "," << current_pos.y() << ")");
                    } else {
                        // Can't move along tangent, need to find the next wall and turn counter-clockwise
                        current_obstacle = next_collision;
                        LOG("Wall follow - BLOCKED, finding next wall IS THIS NTO SAVING???");
                        // Update current obstacle to the one we're now colliding with
                        // Find the closest boundary point from current position to get new wall normal
                        auto [new_boundary_point, new_wall_normal] = findClosestBoundaryAndNormal(current_pos, problem, current_obstacle);
                        
                        // Turn cloclwise from the new wall normal
                        Eigen::Vector2d clockwise_dir = rotateClockwise90(new_wall_normal);
                        Eigen::Vector2d clockwise_pos = current_pos + GLOBAL_STEP_SIZE * clockwise_dir;
                        
                        LOG("New wall normal (" << new_wall_normal.x() << "," << new_wall_normal.y() << ")");
                        LOG("Clockwise dir (" << clockwise_dir.x() << "," << clockwise_dir.y() << ")");
                        LOG("Clockwise pos (" << clockwise_pos.x() << "," << clockwise_pos.y() << ")");
                        
                        current_pos = clockwise_pos;
                        wall_follow_dir = clockwise_dir;
                        wall_normal = new_wall_normal;  // Update to the new wall normal
                        path.waypoints.push_back(current_pos);
                    //    LOG("Wall follow - TURNING COUNTER-CLOCKWISE at (" << current_pos.x() << "," << current_pos.y() << ")");
                    }
                }

                 // Check if this new point is the closest we've seen to the goal
                double current_dist_to_goal = (current_pos - problem.q_goal).norm();
                if (current_dist_to_goal < min_dist_to_goal) {
                    min_dist_to_goal = current_dist_to_goal;
                    closest_point = current_pos;
                }
                
                // Check if we've returned to the starting point of circumnavigation
                if ((current_pos - circumnavigation_start).norm() < GLOBAL_STEP_SIZE * 2 && wall_follow_steps > 5) {
                    LOG("Returned to starting point of circumnavigation - breaking out");
                    LOG("Closest point (" << closest_point.x() << "," << closest_point.y() << ")");
                    std::cout << "Press Enter to continue..." << std::endl;
                    std::cin.get();
                    break;
                }
            }
        }
    }
    return path;
}