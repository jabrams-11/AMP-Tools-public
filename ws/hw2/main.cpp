// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyBugAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /*    Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    /*    Randomly generate the problem     */ 

    // Use WO1 from Exercise 2
    Problem2D problem = HW2::getWorkspace2();

    // Use WO1 from Exercise 2
    /*
    Problem2D problem = HW2::getWorkspace2();
    */

    // Make a random environment spec, edit properties about it such as the number of obstacles
    /*
    Random2DEnvironmentSpecification spec;
    spec.max_obstacle_region_radius = 5.0;
    spec.n_obstacles = 2;
    spec.path_clearance = 0.01;
    spec.d_sep = 0.01;

    //Randomly generate the environment;
    Problem2D problem = EnvironmentTools::generateRandom(spec); // Random environment
    */

    // Declare your algorithm object 
    MyBugAlgorithm algo;
    
    {
        // Call your algorithm on the problem
        amp::Path2D path = algo.plan(problem);

        // Check your path to make sure that it does not collide with the environment 
        bool success = HW2::check(path, problem);

        LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));

        // Visualize the path and environment
        Visualizer::makeFigure(problem, path);
    }

    // Let's get crazy and generate a random environment and test your algorithm
    {
        amp::Path2D path; // Make empty path, problem, and collision points, as they will be created by generateAndCheck()
        amp::Problem2D random_prob; 
        std::vector<Eigen::Vector2d> collision_points;
        
        // Use the most comprehensive version of generateAndCheck to get ALL data
        bool random_trial_success = HW2::generateAndCheck(algo, path, random_prob, collision_points, true, 0u);
        
        LOG("Found valid solution in random environment: " << (random_trial_success ? "Yes!" : "No :("));
        LOG("Path length: " << path.length());
        LOG("Number of waypoints: " << path.waypoints.size());
        LOG("Number of obstacles in random environment: " << random_prob.obstacles.size());
        LOG("Workspace bounds: [" << random_prob.x_min << ", " << random_prob.x_max << "] x [" << random_prob.y_min << ", " << random_prob.y_max << "]");
        LOG("Start position: (" << random_prob.q_init.x() << ", " << random_prob.q_init.y() << ")");
        LOG("Goal position: (" << random_prob.q_goal.x() << ", " << random_prob.q_goal.y() << ")");
        LOG("Number of collision points detected: " << collision_points.size());
        
        // Print collision points if any
        if (!collision_points.empty()) {
            LOG("Collision points:");
            for (size_t i = 0; i < collision_points.size(); i++) {
                LOG("  Collision " << i << ": (" << collision_points[i].x() << ", " << collision_points[i].y() << ")");
            }
        }
        
        // Print path waypoints for detailed analysis
        LOG("Path waypoints:");
        for (size_t i = 0; i < path.waypoints.size(); i++) {
            LOG("  Waypoint " << i << ": (" << path.waypoints[i].x() << ", " << path.waypoints[i].y() << ")");
        }

        // Visualize the path environment, and any collision points with obstacles
        Visualizer::makeFigure(random_prob, path, collision_points);
    }

    Visualizer::saveFigures(true, "hw2_figs");


    HW2::grade(algo, "nonhuman.biologic@myspace.edu", argc, argv);
    
    /* If you want to reconstruct your bug algorithm object every trial (to reset member variables from scratch or initialize), use this method instead*/
    //HW2::grade<MyBugAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, constructor_parameter_1, constructor_parameter_2, etc...);
    
    // This will reconstruct using the default constructor every trial
    //HW2::grade<MyBugAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv);

    return 0;
}