#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <iostream>
#include <vector>
#include <algorithm>

using std::placeholders::_1;

class CompareLikelihoods : public rclcpp::Node
{
public:
  CompareLikelihoods()
      : Node("compare_likelihoods_node")
  {
    this->declare_parameter("num_of_layer", 0);
    this->declare_parameter("interval_ms", 500);

    num_of_layer = get_parameter("num_of_layer").as_int();
    auto interval_ms = get_parameter("interval_ms").as_int();

    if (num_of_layer <= 0)
    {
      RCLCPP_ERROR(get_logger(), "\"num_of_layer\" is not set.");
      rclcpp::shutdown();
    }

    pose_and_likelihoods.clear();
    subscribers.clear();
    for (auto i = 0; i < num_of_layer; i++)
    {
      pose_and_likelihoods.push_back(std::make_pair(pose_type{}, likelihoods_type{}));
      auto subscription = this->create_subscription<subscribe_type>(
          "pose_and_likelihoods" + std::to_string(i), 10,
          [this, i](const subscribe_type::SharedPtr msg)
          {
            this->pose_and_likelihoods[i].first.assign(msg->data.begin(), msg->data.begin() + 3);
            this->pose_and_likelihoods[i].second.assign(msg->data.begin() + 3, msg->data.end());

            this->pose_and_likelihoods[i].first[2] *= 180 / M_PI;
          });
      subscribers.push_back(subscription);
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(interval_ms), std::bind(&CompareLikelihoods::timer_callback, this));
  }

private:
  using subscribe_type = std_msgs::msg::Float64MultiArray;
  using pose_type = std::vector<double>;
  using likelihoods_type = std::vector<double>;

  std::vector<rclcpp::Subscription<subscribe_type>::SharedPtr> subscribers;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::pair<pose_type, likelihoods_type>> pose_and_likelihoods;
  int num_of_layer;

  void timer_callback()
  {
    static size_t but_layer = -1;
    for (size_t i = 0; i < pose_and_likelihoods.size(); i++)
    {
      auto &v = pose_and_likelihoods[i];
      if (v.first.empty() or v.second.empty())
      {
        if (but_layer != i)
        {
          RCLCPP_INFO_STREAM(get_logger(), "Layer " << i << " don't send.");
          but_layer = i;
        }
        return;
      }
    }

    std::vector<std::pair<double, int>> evals;

    RCLCPP_INFO_STREAM(get_logger(), "start compare");
    for (size_t i = 0; i < pose_and_likelihoods.size(); i++)
    {
      auto &likelihoods = pose_and_likelihoods[i].second;
      evals.emplace_back(likelihoods[2], i);
    }

    std::sort(evals.begin(), evals.end());

    for (auto &v : evals)
    {
      auto &likelihoods = pose_and_likelihoods[v.second].second;
      RCLCPP_INFO(get_logger(), "layer %2d: %2.6lf = %2.6lf * %2.6lf", v.second, v.first, likelihoods[0], likelihoods[2]);
    }
    for (auto &v : evals)
    {
      auto i = v.second;
      auto &pose = pose_and_likelihoods[i].first;
      auto &likelihoods = pose_and_likelihoods[i].second;

      std::string likelihoods_str;
      for (auto &l : likelihoods)
      {
        likelihoods_str += std::to_string(l) + ",";
      }
      likelihoods_str.pop_back();

      RCLCPP_INFO(get_logger(), "layer %2d(%3.4f, %3.4f, %3.4f deg): %s", i, pose[0], pose[1], pose[2], likelihoods_str.c_str());
    }

    for (auto &v : pose_and_likelihoods)
    {
      v.first.clear();
      v.second.clear();
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompareLikelihoods>());
  rclcpp::shutdown();

  return 0;
}