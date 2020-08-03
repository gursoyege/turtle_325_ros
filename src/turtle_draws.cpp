#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

class TurtleNode{
public:
  TurtleNode(ros::NodeHandle node) : node_(node), state_(State::Forward){
    publisher_ = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10); // Publishing to turtle velocity control
    subscriber_ = node.subscribe("turtle1/pose", 10, &TurtleNode::turtleCallback, this); //Subscribing to turtle position

    // Setting manually the usual initial position of turtle in case of a miss at the beginning
    pose_message_.x = 5.5;
    pose_message_.y = 5.5;

  }

  virtual void process() {
    switch (state_) {

      case State::Forward: // Move forward
        twist_message_.linear.x = V_FORWARD;
        twist_message_.angular.z = 0;
        publisher_.publish(twist_message_);
        if ( ((pose_message_.x < THRESHOLD_B && pose_message_.y > THRESHOLD_R)
          || (pose_message_.x > THRESHOLD_T && pose_message_.y < THRESHOLD_L)) && can_fix_turn ) {
           state_ = State::TurnFix;
           ROS_INFO("Turning stationary 180 deg");
        }
        else if ( (((pose_message_.x > THRESHOLD_R) && direction == 1)
        || ((pose_message_.x < THRESHOLD_L) && direction == 0)) && can_neg_turn ) {
          state_ = State::TurnNeg;
          ROS_INFO("Turning in negative direction");
        }
        else if ( (((pose_message_.x < THRESHOLD_L) && direction == 1)
        || ((pose_message_.x > THRESHOLD_R) && direction == 0)) && can_pos_turn ) {
          state_ = State::TurnPos;
          ROS_INFO("Turning in positive direction");
        }
        break;

      case State::TurnFix: // Stationary turn
        twist_message_.linear.x = 0;
        twist_message_.angular.z = + V_TURNING_T;
        publisher_.publish(twist_message_);
        can_fix_turn = false;
        direction = !direction;
        state_ = State::AngDecision;
        ROS_INFO("Looking to the current head angle");
          break;

      case State::TurnNeg: // Negative angle turn
        twist_message_.linear.x = V_TURNING_X;
        twist_message_.angular.z = - V_TURNING_T;
        publisher_.publish(twist_message_);
        can_fix_turn = true;
        can_neg_turn = false;
        can_pos_turn = true;
        state_ = State::AngDecision;
        ROS_INFO("5");
          break;

      case State::TurnPos: // Positive angle turn
        twist_message_.linear.x = V_TURNING_X;
        twist_message_.angular.z = + V_TURNING_T;
        publisher_.publish(twist_message_);
        can_fix_turn = true;
        can_neg_turn = true;
        can_pos_turn = false;
        state_ = State::AngDecision;
        ROS_INFO("6");
        break;

      case State::AngDecision: // Look to the head angle of the turtle
        publisher_.publish(twist_message_);
        if (fabsf(pose_message_.theta) < THRESHOLD_DIRECTION){
          state_ = State::ToPi;
          ROS_INFO("Head angle is 0 deg, turning to 180 deg");
        }
        else if (fabsf(pose_message_.theta) > (pi - THRESHOLD_DIRECTION)){
          state_ = State::ToZero;
          ROS_INFO("Head angle is 180 deg, turning to 0 deg");
        }
        break;

      case State::ToPi: // Turn until the head angle is pi
        publisher_.publish(twist_message_);
          if (fabsf(pose_message_.theta) > (pi - THRESHOLD_THETA)) {
            ROS_INFO("At 180 deg, moving forward");
            state_ = State::Forward;
          }
          break;

        case State::ToZero: // Turn until the head angle is zero
          publisher_.publish(twist_message_);
            if (fabsf(pose_message_.theta) < (THRESHOLD_THETA)) {
              ROS_INFO("At 0 deg, moving forward");
              state_ = State::Forward;
            }
            break;
      }
  }

private:
  enum class State { Forward,TurnFix,TurnNeg,TurnPos,AngDecision,ToPi,ToZero};
  State state_;

  bool can_fix_turn = true; // When true, stationary turn is allowed
  bool can_neg_turn = true; // When true, negative angle turn is allowed
  bool can_pos_turn = true; // When true, positive angle turn is allowed

  /* can_xxx_turn vales used to restrict to enter the same state after accomplish a turn.
  Since we're looking to the x coordinate to enter a turning state, when the turtle finishes a turn, it ends up at the same x value.
  For that reason we don't allow to make the same turn for the second time until a different angle turn.*/

  bool direction = 1;

  /* Direction represents the direction of the turtle among the drawn shape.
  1 at the initial position then changing its value after a stationary 180 degrees turn.*/

  const float V_FORWARD = 1; // Linear velocity of the x axis when moving forward
  const float V_TURNING_X = 0.2; // Linear velocity of the x axis when turning
  const float V_TURNING_T = 0.16; // Angular velocity of the z axis when turning
  const float THRESHOLD_R = 9.5; // Below that value is right
  const float THRESHOLD_L = 1.5; // Below that value is left
  const float THRESHOLD_T = 9.5; // Below that value is top
  const float THRESHOLD_B = 1.5; // Below that value is bottom
  const float THRESHOLD_DIRECTION = 0.15; // Sensibility for the direction decision
  const float THRESHOLD_THETA = 0.002; // Sensibility for the theta angle

  const float pi = 3.1415927;

  void turtleCallback(const turtlesim::Pose::ConstPtr& message) {
    //ROS_INFO("Turtle at [%f, %f, %f]",message->x, message->y, message->theta); //Print turtle position
    pose_message_ = *message;
  }

  turtlesim::Pose pose_message_;
  geometry_msgs::Twist twist_message_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  ros::NodeHandle node_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TurtleNode");
  ros::NodeHandle node_;
  ros::Rate loop_rate(5000); // High loop_rate to increase the communication rate thus the precision of turtle position
  TurtleNode master(node_);
  while (ros::ok()) {
      master.process();
      ros::spinOnce();
      loop_rate.sleep();
  }
}
