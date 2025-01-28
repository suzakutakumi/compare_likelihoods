#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <iostream>
#include <vector>
#include <algorithm>
#include <boost/format.hpp>

using std::placeholders::_1;

class CompareLikelihoods : public rclcpp::Node
{
public:
  CompareLikelihoods()
      : Node("compare_likelihoods_node")
  {
    this->declare_parameter("num_of_layer", 0);
    this->declare_parameter("interval_ms", 500);
    this->declare_parameter("select_likelihood", 0);
    this->declare_parameter("surrounding_threshold", 60);
    this->declare_parameter("particle_convergence_threshold", 0.8);

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

    best_pose_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("best_pose", 10);
    best_pose_tmp_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("best_pose_tmp", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(interval_ms), std::bind(&CompareLikelihoods::timer_callback, this));
  }

private:
  using subscribe_type = std_msgs::msg::Float64MultiArray;
  using pose_type = std::vector<double>;
  using likelihoods_type = std::vector<double>;

  std::vector<rclcpp::Subscription<subscribe_type>::SharedPtr> subscribers;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr best_pose_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr best_pose_tmp_publisher;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::pair<pose_type, likelihoods_type>> pose_and_likelihoods;
  int num_of_layer;

  void timer_callback()
  {
    static size_t but_layer = -1;
    for (size_t i = 1; i < pose_and_likelihoods.size(); i++)
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

    std::vector<std::pair<double, int>> select_layers, not_select_layers;

    auto select = get_parameter("select_likelihood").as_int();
    for (size_t i = 0; i < pose_and_likelihoods.size(); i++)
    {
      auto &likelihoods = pose_and_likelihoods[i].second;
      if (pose_and_likelihoods[i].first.empty() or likelihoods.empty())
        continue;

      auto surrounding_threshold = get_parameter("surrounding_threshold").as_int();
      auto particle_convergence_threshold = get_parameter("particle_convergence_threshold").as_double();

      if (likelihoods[0] >= particle_convergence_threshold and likelihoods[4] >= (double)surrounding_threshold)
      {
        select_layers.emplace_back(likelihoods[select], i);
      }
      else
      {
        not_select_layers.emplace_back(likelihoods[select], i);
      }
    }

    std::sort(select_layers.begin(), select_layers.end());
    std::sort(not_select_layers.begin(), not_select_layers.end());

    std::string output = "";

    for (auto &v : not_select_layers)
    {
      auto i = v.second;
      auto &pose = pose_and_likelihoods[i].first;
      auto &likelihoods = pose_and_likelihoods[i].second;

      output += "\n";
      output += (boost::format("layer %2d(%7.4f, %7.4f, %8.4f deg)") % i % pose[0] % pose[1] % pose[2]).str();
      for (auto &l : likelihoods)
      {
        output += (boost::format("%10.5f, ") % l).str();
      }
    }
    output += "\n----";

    for (auto &v : select_layers)
    {
      auto i = v.second;
      auto &pose = pose_and_likelihoods[i].first;
      auto &likelihoods = pose_and_likelihoods[i].second;

      output += "\n";
      output += (boost::format("layer %2d(%7.4f, %7.4f, %8.4f deg)") % i % pose[0] % pose[1] % pose[2]).str();
      for (auto &l : likelihoods)
      {
        output += (boost::format("%10.5f, ") % l).str();
      }
    }
    RCLCPP_INFO(get_logger(), output);

    auto pose_iter = select_layers.rbegin();
    auto best_pose_index = 0;
    if (pose_iter != select_layers.rend() and (pose_iter->second != 0 or (++pose_iter) != select_layers.rend()))
    {
      best_pose_index = pose_iter->second;
      RCLCPP_INFO(get_logger(), "publish best pose");
      std_msgs::msg::Float64MultiArray::UniquePtr best_pose(new std_msgs::msg::Float64MultiArray);
      best_pose->data = pose_and_likelihoods[best_pose_index].first;
      best_pose->data[2] *= M_PI / 180.0;
      best_pose_publisher->publish(std::move(best_pose));
    }

    std_msgs::msg::Float64MultiArray::UniquePtr best_pose_tmp(new std_msgs::msg::Float64MultiArray);
    if (best_pose_index != 0)
    {
      best_pose_tmp->data = pose_and_likelihoods[best_pose_index].first;
    }
    else
    {
      best_pose_tmp->data = std::vector<double>{NAN, NAN, NAN};
    }
    best_pose_tmp->data.push_back(best_pose_index);
    best_pose_tmp_publisher->publish(std::move(best_pose_tmp));

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