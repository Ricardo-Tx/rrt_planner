#include <rrt_planner/collision_detector.h>

namespace rrt_planner {

    CollisionDetector::CollisionDetector(costmap_2d::Costmap2DROS* costmap) {
        costmap_ = costmap->getCostmap();

        resolution_ = costmap_->getResolution();
        origin_x_ = costmap_->getOriginX();
        origin_y_ = costmap_->getOriginY();
    }

    bool CollisionDetector::inFreeSpace(const double* world_pos) {
        unsigned int mx, my;

        // Convert world coordinates to map coordinates
        if (!costmap_->worldToMap(world_pos[0], world_pos[1], mx, my)) {
            return false;  // If outside bounds
        }

        unsigned char cost = costmap_->getCost(mx, my);

        return cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE;  // Adjust threshold based on your requirements
    }

    bool CollisionDetector::obstacleBetween(const double* point_a, const double* point_b) {
        double dist = computeDistance(point_a, point_b);

        if (dist < resolution_) {
            return !inFreeSpace(point_b);
        } else {
            int num_steps = static_cast<int>(floor(dist / resolution_));

            double point_i[2];
            double t = 0.0; // lerp parameter
            for (int n = 1; n <= num_steps; n++) {
                t = n / num_steps;
                point_i[0] = point_a[0] + (point_b[0] - point_a[0]) * t;
                point_i[1] = point_a[1] + (point_b[1] - point_a[1]) * t;

                if ( !inFreeSpace(point_i) ) return true;
            }
            
            return false;
        }
    }

};