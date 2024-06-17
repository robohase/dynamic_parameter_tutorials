#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class DynamicTutorialsNode : public rclcpp::Node
{
public:
  DynamicTutorialsNode()
  : Node("dynamic_tutorials_node"),
    int_param(0),
    double_param(0.0),
    str_param(""),
    bool_param(false),
    size(1)
  {
    // パラメータの宣言と初期化
    this->declare_parameter<int>("int_param", int_param);
    this->declare_parameter<double>("double_param", double_param);
    this->declare_parameter<std::string>("str_param", str_param);
    this->declare_parameter<bool>("bool_param", bool_param);
    this->declare_parameter<int>("size", size);
    
    // パラメータの取得
    this->get_parameter("int_param", int_param);
    this->get_parameter("double_param", double_param);
    this->get_parameter("str_param", str_param);
    this->get_parameter("bool_param", bool_param);
    this->get_parameter("size", size);

    auto parameter_change_cb = std::bind(&DynamicTutorialsNode::parameter_callback, this, std::placeholders::_1);
    reset_param_handler_ = this->add_on_set_parameters_callback(parameter_change_cb);
  }

rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  for (const auto &parameter : parameters) {
    if (parameter.get_name() == "int_param") {
      int_param = parameter.as_int();
      RCLCPP_INFO(this->get_logger(), "int_param changed to %d", int_param);
    } else if (parameter.get_name() == "double_param") {
      double_param = parameter.as_double();
      RCLCPP_INFO(this->get_logger(), "double_param changed to %f", double_param);
    } else if (parameter.get_name() == "str_param") {
      str_param = parameter.as_string();
      RCLCPP_INFO(this->get_logger(), "str_param changed to %s", str_param.c_str());
    } else if (parameter.get_name() == "bool_param") {
      bool_param = parameter.as_bool();
      RCLCPP_INFO(this->get_logger(), "bool_param changed to %s", bool_param ? "true" : "false");
    } else if (parameter.get_name() == "size") {
      size = parameter.as_int();
      RCLCPP_INFO(this->get_logger(), "size changed to %d", size);
    }
  }
  return result;
}

private:
  int int_param;
  double double_param;
  std::string str_param;
  bool bool_param;
  int size;
  OnSetParametersCallbackHandle::SharedPtr reset_param_handler_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamicTutorialsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

