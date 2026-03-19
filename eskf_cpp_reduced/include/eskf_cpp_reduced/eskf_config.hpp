/**
 * @file eskf_config.hpp
 * @brief YAML configuration file loader for ESKF
 */

#pragma once

#include "eskf_types.hpp"
#include <string>

namespace eskf {

/**
 * @brief Load ESKF parameters from YAML configuration file
 * 
 * @param filename Path to YAML configuration file
 * @return ESKFParams structure with loaded values
 * @throws std::runtime_error if file cannot be read or parsed
 */
ESKFParams loadConfig(const std::string& filename);

/**
 * @brief Load ESKF parameters from YAML string
 * 
 * @param yaml_content YAML content as string
 * @return ESKFParams structure with loaded values
 */
ESKFParams loadConfigFromString(const std::string& yaml_content);

/**
 * @brief Print configuration summary to console
 * @param params Parameters to print
 */
void printConfig(const ESKFParams& params);

} // namespace eskf
