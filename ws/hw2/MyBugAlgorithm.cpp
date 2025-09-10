#include "MyBugAlgorithm.h"
#include <iostream> // For error messages

// Define a global (file-static) step size
static constexpr double GLOBAL_STEP_SIZE = 0.01;
static constexpr double WALL_OFFSET = 0.05; // Keep bug this distance away from walls
static constexpr double LOOKAHEAD_STEP = 5;
static constexpr double EXTRA_STEP = 1.0;

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
// Returns (boundary_point, wall_normal, vertex_index) where vertex_index is the starting vertex for clockwise traversal
std::tuple<Eigen::Vector2d, Eigen::Vector2d, int> MyBugAlgorithm::findClosestBoundaryAndNormal(const Eigen::Vector2d& point, const amp::Problem2D& problem, int obstacle_index) {
    double min_distance = std::numeric_limits<double>::max();
    Eigen::Vector2d closest_point = point;
    Eigen::Vector2d wall_normal = Eigen::Vector2d(1, 0); // Default normal
    int closest_vertex_index = 0; // Which vertex to start clockwise traversal from
    
    // Only check the specified obstacle
    if (obstacle_index >= 0 && obstacle_index < problem.obstacles.size()) {
        // Get vertices in clockwise order
        std::vector<Eigen::Vector2d> vertices = getVerticesClockwise(problem, obstacle_index);
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
                //LOG("Distance: " << distance);
                //LOG("EDGE VERTICES: " << v1.x() << "," << v1.y() << " and " << v2.x() << "," << v2.y());
                min_distance = distance;
                closest_point = closest_on_edge;
                closest_vertex_index = (i+1) % n; // Store the vertex index for clockwise traversal
                //LOG("Closest vertex index: " << closest_vertex_index);
                // Print the edge that was chosen
                //LOG("Chosen vertex vector: " << vertices[closest_vertex_index].x() << "," << vertices[closest_vertex_index].y());
                //LOG("Next vertex vector: " << vertices[(closest_vertex_index+1) % n].x() << "," << vertices[(closest_vertex_index+1) % n].y());
                
                // Print all vertices in the list
                //LOG("All vertices in clockwise order:");
                
                // Calculate outward normal for clockwise polygon
                // For edge v1->v2 in clockwise order, outward normal is perpendicular to the LEFT
                Eigen::Vector2d edge_unit = edge / edge_length;
                wall_normal = Eigen::Vector2d(edge_unit.y(), -edge_unit.x()); // Left perpendicular = outward normal for clockwise
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
    
    return std::make_tuple(closest_point, wall_normal.normalized(), closest_vertex_index);
}

// Helper method to get vertices in clockwise order (reverse of CCW)
std::vector<Eigen::Vector2d> MyBugAlgorithm::getVerticesClockwise(const amp::Problem2D& problem, int obstacle_index) {
    std::vector<Eigen::Vector2d> clockwise_vertices;
    
    if (obstacle_index >= 0 && obstacle_index < problem.obstacles.size()) {
        const auto& vertices_ccw = problem.obstacles[obstacle_index].verticesCCW();
        // Reverse the CCW vertices to get clockwise order
        for (int i = vertices_ccw.size() - 1; i >= 0; i--) {
            clockwise_vertices.push_back(vertices_ccw[i]);
        }
    }
    
    return clockwise_vertices;
}

// The fully implemented Bug 1 algorithm
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    Eigen::Vector2d current_pos = problem.q_init;
    path.waypoints.push_back(current_pos);

    int max_steps = 50000; // Prevent infinite loops REMOVE LATER
    int step_count = 0;

    // Main loop: Continue until the goal is reached
    while ((current_pos - problem.q_goal).norm() > GLOBAL_STEP_SIZE) {
        step_count++;
        
        // === STATE 1: MOVE TOWARDS GOAL ===
        Eigen::Vector2d goal_direction = (problem.q_goal - current_pos).normalized();

        Eigen::Vector2d next_pos = current_pos + GLOBAL_STEP_SIZE * goal_direction;
        Eigen::Vector2d next_pos_lookahead = current_pos + LOOKAHEAD_STEP * GLOBAL_STEP_SIZE * goal_direction;

        // Check for collision
        int collision_obstacle = isInCollision(next_pos_lookahead, problem);
        if (collision_obstacle == -1) {
            // No collision, take the step
            current_pos = next_pos;
            path.waypoints.push_back(current_pos);
        } else {
            // Collision detected! Enter wall-following mode.
            // === STATE 2: CIRCUMNAVIGATE OBSTACLE (Right-Hand Rule) === amend infuture follow clockwise iterate thru points on the obstacle   ??? keep checkign forward vector for collisions with other obstacles
            Eigen::Vector2d hit_point = current_pos; // this might need to be changed to the point on the obstacle boundary that we hit
            Eigen::Vector2d closest_point = hit_point;
            Eigen::Vector2d circumnavigation_start = hit_point; // Save starting position for circumnavigation

            double min_dist_to_goal = (hit_point - problem.q_goal).norm();

            // Use the obstacle we just hit
            int current_obstacle = collision_obstacle;
            //LOG("Hit obstacle index: " << current_obstacle);

            // Find the closest point on the obstacle boundary AND get wall normal
            auto [boundary_point, wall_normal, start_vertex_index] = findClosestBoundaryAndNormal(current_pos, problem, current_obstacle);
            //LOG("Start vertex index: " << start_vertex_index);
            std::vector<Eigen::Vector2d> clockwise_vertices = getVerticesClockwise(problem, current_obstacle);
            //LOG("Start vertex: " << clockwise_vertices[start_vertex_index].x() << "," << clockwise_vertices[start_vertex_index].y());
            //LOG("Next vertex: " << clockwise_vertices[(start_vertex_index + 1) % clockwise_vertices.size()].x() << "," << clockwise_vertices[(start_vertex_index + 1) % clockwise_vertices.size()].y());
           
  
            // Get vertices in clockwise order for wall following
        
            int current_vertex_index = start_vertex_index;
            
            // Create offset target using wall normal (much simpler!)
            // Offset should be from the start vertex, not the arbitrary boundary point
            Eigen::Vector2d start_vertex = clockwise_vertices[current_vertex_index];
            Eigen::Vector2d target_vertex = start_vertex + WALL_OFFSET * wall_normal;
            
            // Track if we've completed circumnavigation
            bool circumnavigated = false;
            
            // Wall following loop - go clockwise around obstacle
            int wall_follow_steps = 0;
            while (true) {
    
                // Check if current position is closer to goal than previous minimum
                double current_dist_to_goal = (current_pos - problem.q_goal).norm();
                if (current_dist_to_goal < min_dist_to_goal) {
                    min_dist_to_goal = current_dist_to_goal;
                    closest_point = current_pos;
                    //LOG("New closest point to goal found at distance: " << min_dist_to_goal);
                    
                }
                // if (circumnavigated) {
                //     std::cout << "Press Enter to continue..." << std::endl;
                //     std::cin.get();
                // }
               // LOG("Cloest point to goal: " << closest_point.x() << "," << closest_point.y());
               
                    // We can move towards the goal! Check if we've completed circumnavigation
                 if (circumnavigated && (current_pos - closest_point).norm() < GLOBAL_STEP_SIZE * 5) {
                        // We've completed circumnavigation AND we're at the closest point to goal
                        //LOG("Circumnavigation complete and at closest point - can leave wall following!");
                        //LOG("Moving towards goal from closest point");
                        break; // Exit wall following mode
                    }
                
               
                wall_follow_steps++;
               //LOG("location of bug: " << current_pos.x() << "," << current_pos.y());
                // std::cout << "Press Enter to continue..." << std::endl;
                // std::cin.get();
                // Move towards the current target vertex in clockwise direction
                Eigen::Vector2d vertex_direction = (target_vertex - current_pos).normalized();
                Eigen::Vector2d vertex_step = current_pos + GLOBAL_STEP_SIZE * vertex_direction;
                // std::cout << "Press Enter to continue..." << std::endl;
                // std::cin.get();
                // Check 50% further ahead to anticipate collisions
                Eigen::Vector2d lookahead_step = current_pos + LOOKAHEAD_STEP * GLOBAL_STEP_SIZE * vertex_direction;
                
                // Check if we can move towards the vertex (check both current step and lookahead)
                int lookahead_collision = isInCollision(lookahead_step, problem);
               // LOG("Lookahead collision: " << lookahead_collision);
               // LOG("lookahead step vector: " << lookahead_step.x() << "," << lookahead_step.y());
         
                if (lookahead_collision == -1) {
                    current_pos = vertex_step;
                    path.waypoints.push_back(current_pos);
                   // LOG("Moving towards vertex " << current_vertex_index << " at (" << current_pos.x() << "," << current_pos.y() << ")");
                   // LOG("target: " << target_vertex.x() << "," << target_vertex.y());
                    
                    // Check if we've reached the target vertex
                    if ((current_pos - target_vertex).norm() < GLOBAL_STEP_SIZE) {
                        // Move to the next vertex in clockwise order
                        // Take an extra step past the vertex to ensure we go past it
                        current_pos = current_pos + GLOBAL_STEP_SIZE* EXTRA_STEP * vertex_direction;
                        path.waypoints.push_back(current_pos);
                        
                        current_vertex_index = (current_vertex_index + 1) % clockwise_vertices.size();
                    
                     
                        // Get wall normal for the next edge (from current_vertex_index to next vertex, i.e., current_vertex_index + 1)
                        int next_vertex_index = (current_vertex_index - 1) % clockwise_vertices.size();
                        Eigen::Vector2d edge = clockwise_vertices[next_vertex_index] - clockwise_vertices[current_vertex_index];
                        // Print the edge vertices

                        // Compute outward normal (assuming vertices are ordered counter-clockwise)
                        Eigen::Vector2d next_wall_normal(edge.y(), -edge.x());
                        next_wall_normal.normalize();
                        start_vertex = clockwise_vertices[current_vertex_index];
                        target_vertex = start_vertex + WALL_OFFSET * next_wall_normal;
                
                        
                  
                        
                        
                        //LOG("Reached vertex, moving to next vertex " << current_vertex_index << " at offset position (" << target_vertex.x() << "," << target_vertex.y() << ")");
                        //LOG("Raw vertex position: (" << clockwise_vertices[current_vertex_index].x() << "," << clockwise_vertices[current_vertex_index].y() << ")");
                    }
                } else {
                    // Hit another obstacle, break out of wall following
                    //LOG("Hit another obstacle while wall following, obstacle index: " << lookahead_collision);
                  
                    // Print vertices of the obstacle we hit
                    std::vector<Eigen::Vector2d> hit_obstacle_vertices = getVerticesClockwise(problem, lookahead_collision);
                    //LOG("Hit obstacle " << lookahead_collision << " vertices:");
                    for (size_t i = 0; i < hit_obstacle_vertices.size(); i++) {
                        //LOG("  Vertex " << i << ": (" << hit_obstacle_vertices[i].x() << "," << hit_obstacle_vertices[i].y() << ")");
                    }
                    
                    // Switch to the new obstacle and start circumnavigating it
                    current_obstacle = lookahead_collision;
                    
                    // Find the closest point on the new obstacle boundary and get wall normal
                    auto [new_boundary_point, new_wall_normal, new_start_vertex_index] = findClosestBoundaryAndNormal(current_pos, problem, current_obstacle);
                    //LOG("New start vertex index: " << new_start_vertex_index);
                    clockwise_vertices = getVerticesClockwise(problem, current_obstacle);
                    //LOG("New start vertex: " << clockwise_vertices[new_start_vertex_index].x() << "," << clockwise_vertices[new_start_vertex_index].y());
                    //LOG("New next vertex: " << clockwise_vertices[(new_start_vertex_index + 1) % clockwise_vertices.size()].x() << "," << clockwise_vertices[(new_start_vertex_index + 1) % clockwise_vertices.size()].y());
               
                    // Update current vertex index and target vertex for the new obstacle
                    current_vertex_index = new_start_vertex_index;
                    
                    // Create offset target using the new wall normal (much simpler!)
                    start_vertex = clockwise_vertices[current_vertex_index];
                    target_vertex = start_vertex + WALL_OFFSET * new_wall_normal;
                    //LOG("New target vertex: " << target_vertex.x() << "," << target_vertex.y());
           
                }
                //LOG("current position: " << current_pos.x() << "," << current_pos.y());
                //LOG("circumnavigation start: " << circumnavigation_start.x() << "," << circumnavigation_start.y());
                //LOG("wall follow steps: " << wall_follow_steps);
                // std::cout << "Press Enter to continue..." << std::endl;
                // std::cin.get();
                // Check if we've completed a full loop around the obstacle
                if ((current_pos - circumnavigation_start).norm() < GLOBAL_STEP_SIZE * 5 && wall_follow_steps > 15 && circumnavigated == false) {
                    circumnavigated = true;

                }
            }
            
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          
          /*
       
          
            // For clockwise wall following, we want to move perpendicular to the normal
            // (tangent direction). For CCW polygons, outward normal points away from obstacle
            // So tangent = rotate normal 90 degrees clockwise
            Eigen::Vector2d wall_follow_dir = rotateClockwise90(wall_normal);
            
            LOG("Hit obstacle at (" << current_pos.x() << "," << current_pos.y() << ")");
            LOG("Boundary point (" << boundary_point.x() << "," << boundary_point.y() << ")");
            LOG("Wall normal (" << wall_normal.x() << "," << wall_normal.y() << ")");
            LOG("Wall follow dir (" << wall_follow_dir.x() << "," << wall_follow_dir.y() << ")");
            std::cout << "Press Enter to continue..." << std::endl;
            std::cin.get();
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
                        auto [new_boundary_point, new_wall_normal, new_vertex_index] = findClosestBoundaryAndNormal(current_pos, problem, current_obstacle);
                        current_vertex_index = new_vertex_index; // Update current vertex index
                        
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
                */
        }
            
    }
    return path;
}