#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
    
    private:
        // Helper method for collision detection - returns obstacle index or -1 if no collision
        int isInCollision(const Eigen::Vector2d& point, const amp::Problem2D& problem);
        
        // Helper methods for vector rotation
        Eigen::Vector2d rotateClockwise90(const Eigen::Vector2d& vec);
        Eigen::Vector2d rotateCounterClockwise90(const Eigen::Vector2d& vec);
        
        // Helper methods for surface normal detection - returns (boundary_point, wall_normal, vertex_index)
        std::tuple<Eigen::Vector2d, Eigen::Vector2d, int> findClosestBoundaryAndNormal(const Eigen::Vector2d& point, const amp::Problem2D& problem, int obstacle_index);
        
        // Helper method to get vertices in clockwise order
        std::vector<Eigen::Vector2d> getVerticesClockwise(const amp::Problem2D& problem, int obstacle_index);
        
        // Add any member variables here...
};