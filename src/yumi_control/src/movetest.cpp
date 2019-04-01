/* Author: Lvhonghao*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

void randgo()
{
  // 连接move_group节点中的机械臂实例组，这里的组名arm是我们之前在setup assistant中设置的
  moveit::planning_interface::MoveGroupInterface group("right_arm");
  // 随机产生一个目标位置
  group.setRandomTarget();
  // 开始运动规划，并且让机械臂移动到目标位置
  group.move();
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movetest", ros::init_options::AnonymousName);
  // 创建一个异步的自旋线程（spinning thread）
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while (1)
  {
  randgo();
  }
  ros::waitForShutdown();
}