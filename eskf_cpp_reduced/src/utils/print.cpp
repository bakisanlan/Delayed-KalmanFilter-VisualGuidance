/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "eskf_cpp_reduced/utils/print.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>

using namespace eskf;

// Helper function to get current timestamp string
static std::string getTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
  
  std::tm tm_now;
#ifdef _WIN32
  localtime_s(&tm_now, &time_t_now);
#else
  localtime_r(&time_t_now, &tm_now);
#endif
  
  std::ostringstream oss;
  oss << std::setfill('0') 
      << std::setw(2) << tm_now.tm_hour << ":"
      << std::setw(2) << tm_now.tm_min << ":"
      << std::setw(2) << tm_now.tm_sec << "."
      << std::setw(3) << ms.count();
  return oss.str();
}

// Need to define the static variable for everything to work
Printer::PrintLevel Printer::current_print_level = PrintLevel::INFO;
std::FILE* Printer::log_file = nullptr;

void Printer::setPrintLevel(const std::string &level) {
  if (level == "ALL")
    setPrintLevel(PrintLevel::ALL);
  else if (level == "DEBUG")
    setPrintLevel(PrintLevel::DEBUG);
  else if (level == "INFO")
    setPrintLevel(PrintLevel::INFO);
  else if (level == "WARNING")
    setPrintLevel(PrintLevel::WARNING);
  else if (level == "ERROR")
    setPrintLevel(PrintLevel::ERROR);
  else if (level == "SILENT")
    setPrintLevel(PrintLevel::SILENT);
  else {
    std::cout << "Invalid print level requested: " << level << std::endl;
    std::cout << "Valid levels are: ALL, DEBUG, INFO, WARNING, ERROR, SILENT" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

void Printer::setPrintLevel(PrintLevel level) {
  Printer::current_print_level = level;
  std::cout << "Setting printing level to: ";
  switch (current_print_level) {
  case PrintLevel::ALL:
    std::cout << "ALL";
    break;
  case PrintLevel::DEBUG:
    std::cout << "DEBUG";
    break;
  case PrintLevel::INFO:
    std::cout << "INFO";
    break;
  case PrintLevel::WARNING:
    std::cout << "WARNING";
    break;
  case PrintLevel::ERROR:
    std::cout << "ERROR";
    break;
  case PrintLevel::SILENT:
    std::cout << "SILENT";
    break;
  default:
    std::cout << std::endl;
    std::cout << "Invalid print level requested: " << level << std::endl;
    std::cout << "Valid levels are: ALL, DEBUG, INFO, WARNING, ERROR, SILENT" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  std::cout << std::endl;
}

void Printer::setLogFile(const std::string &filepath) {
  if (log_file) {
    std::fclose(log_file);
  }
  log_file = std::fopen(filepath.c_str(), "w");
  if (!log_file) {
    std::cerr << "Printer: Failed to open log file " << filepath << std::endl;
  }
}

void Printer::closeLogFile() {
  if (log_file) {
    std::fclose(log_file);
    log_file = nullptr;
  }
}

void Printer::debugPrint(PrintLevel level, const char location[], const char line[], const char *format, ...) {
  // Only print for the current debug level
  if (static_cast<int>(level) < static_cast<int>(Printer::current_print_level)) {
    return;
  }

  std::string timestamp = getTimestamp();

  // Print timestamp first
  printf("[%s] ", timestamp.c_str());
  if (log_file) {
    fprintf(log_file, "[%s] ", timestamp.c_str());
  }

  // Print the location info first for our debug output
  // Truncate the filename to the max size for the filepath
  if (static_cast<int>(Printer::current_print_level) <= static_cast<int>(Printer::PrintLevel::DEBUG)) {
    std::string path(location);
    std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
    std::string loc_str;
    if (base_filename.size() > MAX_FILE_PATH_LEGTH) {
      loc_str = base_filename.substr(base_filename.size() - MAX_FILE_PATH_LEGTH, base_filename.size());
    } else {
      loc_str = base_filename;
    }
    printf("%s:%s ", loc_str.c_str(), line);
    if (log_file) {
      fprintf(log_file, "%s:%s ", loc_str.c_str(), line);
    }
  }

  // Print the rest of the args
  va_list args;
  va_start(args, format);
  
  if (log_file) {
    va_list args_copy;
    va_copy(args_copy, args);
    vfprintf(log_file, format, args_copy);
    fflush(log_file);
    va_end(args_copy);
  }
  
  vprintf(format, args);
  va_end(args);
  
  // Flush immediately to avoid buffering delays
  fflush(stdout);
}
