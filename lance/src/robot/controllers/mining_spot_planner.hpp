#include <functional>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdexcept>


/*Assumptions that are made and the general philosophy:
    If the excavation zone is a rectangle, x is the shorter side
        This is so that the mining strips are as long as possible 
            (Although it is possible shorter strips are found to be more effective which would reverse this)

        This also effects the front_to_center_mining_offset
            Depending on what direction you are mining in this should be applied to a different coordinate direction
            This is because the traversal controller will put the center of the robot at the point 
            Conversely, the perception node will do it's measurements for mining distance from the exact point requested
            so the mining distance would be too far by about half the robot length causing the robot to collide with the obstacle with the offset
        
*/

// Robot Params
struct RobotInfo
{
    const float track_Width = 0.75;
    const float full_Width = 1.0;
    const float track_to_center = full_Width/2;
    // front_to_center_mining_offset is the offset since traversal controller measures based on the center of the robot but the perception node measures from the front of the robot
        // Currently arbitrary value of 0.35 
    const float front_to_center_mining_offset = 0.35;
    const float minimum_accepted_mining_distance = 0.25; // anything less than this will cause the controller to just increment since it is too little 
    const float invalid_mining_offset = 0.1;
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
            
            // Call the perception node with the start pos
                // 90 is in degrees to have the heading north (assuming that is how the coordinates are configured)
            float mining_distance = perception_eval(start_mining_point.x(), start_mining_point.y(), 90.f);
            

            int iteration_number = 0; // In theory, there should always be somewhere to mine unless there was a literal wall of rocks but for testing this is useful
            // Repeat this until the mining distance is greater minimum_accepted_mining_distance of a meter or a "reasonable" mining distance to be determined
                // Depending on how costly perception_eval is to run, the offset vector could be adjusted
                // Optimally it is as small as possible to reduce the wasted area around rocks/obstacles
            while (mining_distance < robot_info.minimum_accepted_mining_distance) {
                iteration_number ++;
                if (iteration_number > 50){// arbitrary, but this seems like this would be too many calls in a row so just kill the program in an expected way
                    throw std::runtime_error("Tried to find operation more than 50 times, giving up now )=");
                }
                start_mining_point += Eigen::Vector2f(robot_info.invalid_mining_offset,0);// move it slightly by the offset
                mining_distance = perception_eval(start_mining_point.x(), start_mining_point.y(), 90.f); // Keep doing perception calls until a reasonable distance is found
                
                
                // prevent going out of the mining x bounds - reset to default position
                if (start_mining_point.x() > mining_zone_bounds.max().x()-robot_info.track_to_center){
                    start_mining_point = mining_zone_bounds.min() + Eigen::Vector2f(robot_info.track_to_center, 0);
                }
            }
            // Assign the end_mining_point with the mining_distance from the perception call
            Eigen::Vector2f end_mining_point = start_mining_point + Eigen::Vector2f(0,mining_distance);

            std::cout <<  mining_zone_bounds.max().y()<< " : " << end_mining_point.y()<< "\n";


            if (end_mining_point.y() > mining_zone_bounds.max().y()){
                end_mining_point = Eigen::Vector2f(end_mining_point.x(),mining_zone_bounds.max().y());
            }

            last_start_point = start_mining_point;

            return std::pair<Eigen::Vector2f,Eigen::Vector2f>(start_mining_point,end_mining_point);
            
        }
    protected:
        
    
        Eigen::Vector2f getStartingTestPoint() {
            // If the starting Test point here wasn't
            if (last_start_point.x() < mining_zone_bounds.min().x() + robot_info.track_to_center) {
                // Move a full robot width over
                last_start_point += Eigen::Vector2f(robot_info.track_to_center,0);
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
                last_start_point = mining_zone_bounds.min() + Eigen::Vector2f(robot_info.track_to_center,0);
            }

            return last_start_point;
        }

};
