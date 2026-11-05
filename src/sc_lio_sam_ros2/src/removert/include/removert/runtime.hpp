#pragma once
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <type_traits>

namespace lt_mapper
{
inline rclcpp::Node::SharedPtr node;

template<class T>
void parameter(const std::string &name, T &value, const T &fallback)
{
    if (!node->has_parameter(name))
        node->declare_parameter(name, fallback);
    node->get_parameter(name, value);
}

inline void parameter(const std::string &name, std::string &value, const char *fallback)
{
    parameter(name, value, std::string(fallback));
}

inline void
parameter(const std::string &name, double &value, double fallback)
{
    if (!node->has_parameter(name))
    {
        const auto &overrides = node->get_node_parameters_interface()->get_parameter_overrides();
        const auto entry = overrides.find(name);
        node->declare_parameter(name, entry == overrides.end() ?
            rclcpp::ParameterValue(fallback) : entry->second);
    }
    auto parameter = node->get_parameter(name);
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        value = static_cast<double>(parameter.as_int());
    else
        value = parameter.as_double();
}

inline void parameter(const std::string &name, float &value, double fallback)
{
    double result;
    parameter(name, result, fallback);
    value = static_cast<float>(result);
}

inline void parameter(const std::string &name, std::vector<float> &value,
                      const std::vector<float> &fallback)
{
    std::vector<double> result;
    parameter(name, result, std::vector<double>(fallback.begin(), fallback.end()));
    value.assign(result.begin(), result.end());
}

// Output directories must not contain either input session.
inline std::filesystem::path output_directory(const std::string &text,
                                               const std::vector<std::string> &inputs)
{
    namespace fs = std::filesystem;
    if (text.empty())
        throw std::runtime_error("Output directory must be specified");
    auto output = fs::weakly_canonical(fs::absolute(text));
    if (output == output.root_path())
        throw std::runtime_error("Output directory cannot be the filesystem root");
    std::vector<std::string> protected_paths = inputs;
    protected_paths.push_back(fs::current_path().string());
    if (const char *home = std::getenv("HOME"))
        protected_paths.emplace_back(home);
    for (const auto &input : protected_paths)
    {
        if (input.empty())
            continue;
        auto path = fs::weakly_canonical(fs::absolute(input));
        auto relative = path.lexically_relative(output);
        if (relative.empty() || *relative.begin() != "..")
            throw std::runtime_error("Output directory must not contain an input session");
    }
    return output;
}
}
