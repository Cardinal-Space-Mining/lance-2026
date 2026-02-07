#include <functional>
#include <eigen3/Eigen/Geometry>

struct RobotInfo
{
    float track_Width = 0.75;
    float full_Width = 1.0;
};


using Box2f = Eigen::AlignedBox2f;


class MiningSpotPlanner {
    std::function<float(float, float, float)> perception_eval;
    const RobotInfo robot_info;
    const Box2f mining_zone_bounds;

    Eigen::Vector2f last_start_point;

    public:
        MiningSpotPlanner(std::function<float(float, float, float)> perception_eval, RobotInfo robot_info, Box2f mining_zone_bounds) :
         perception_eval(perception_eval), 
         robot_info{robot_info},
         mining_zone_bounds{mining_zone_bounds}
        
        {
            last_start_point = mining_zone_bounds.min();

        }
        
        std::pair<Eigen::Vector2f,Eigen::Vector2f> getMiningSpot(){
        
            Eigen::Vector2f start_mining_point = getStartingTestPoint();
            
            // 90 is in degrees to have the heading north (assuming that is how the coordinates are configured)
            float mining_distance = perception_eval(start_mining_point.x(), start_mining_point.y(), 90.f);
            // Repeat this until the mining distance is greater than a half a meter or a "reasonable" mining distance to be determined
                // Depending on how costly perception_eval is to run, the offset vector could be adjusted
                // Optimally it is as small as possible to reduce the wasted area around rocks/obstacles
            while (mining_distance < 0.5) { 
                mining_distance = perception_eval(start_mining_point.x(), start_mining_point.y(), 90.f); // Keep doing perception calls until a reasonable distance is found
                start_mining_point += Eigen::Vector2f(0.1,0);
            }

            Eigen::Vector2f end_mining_point = start_mining_point + Eigen::Vector2f(0,mining_distance);
            
            return std::pair<Eigen::Vector2f,Eigen::Vector2f>(start_mining_point,end_mining_point);
            
        }
    protected:
        
    
        Eigen::Vector2f getStartingTestPoint() {
            // If the starting Test point here wasn't
            // Eigen::Vector2f starting_test_point = mining_zone_bounds.min();
            if (last_start_point.x() < robot_info.track_Width) {
                // Move a full robot width over
                last_start_point += Eigen::Vector2f(robot_info.full_Width,0);
            }
            else{
                // Move it 1 track width over
                last_start_point += Eigen::Vector2f(robot_info.track_Width,0);
            }
            
            // For right now just reset to the corner again
                // In the future the robot could start on the other side of the mining zone
                // This would enable spots that were previously blocked by obstacles to still be mined
                // Or even better have really smart mapping where sections are planned out based on the obstacles with a matrix
            if (last_start_point.x() > mining_zone_bounds.max().x()){
                last_start_point = mining_zone_bounds.min() + Eigen::Vector2f(robot_info.full_Width,0);
            }

            return last_start_point;
        }

};