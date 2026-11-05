#include "removert/Removerter.h"

int
main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    lt_mapper::node = std::make_shared<rclcpp::Node>("removert");
    int result = 0;
    try
    {
        ltremovert::Removerter processor;
        processor.run();
        RCLCPP_INFO(lt_mapper::node->get_logger(), "Processing completed");
        bool keep_alive = true;
        lt_mapper::parameter("keep_alive", keep_alive, true);
        if (keep_alive)
            rclcpp::spin(lt_mapper::node);
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(lt_mapper::node->get_logger(), "%s", error.what());
        result = 1;
    }
    lt_mapper::node.reset();
    rclcpp::shutdown();
    return result;
}
