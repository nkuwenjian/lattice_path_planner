#ifndef LATTICE_PATH_PLANNER_SBPL_EXCEPTION_H
#define LATTICE_PATH_PLANNER_SBPL_EXCEPTION_H

#include <ros/console.h>
#include <stdexcept>

namespace lattice_path_planner
{
class SBPL_Exception : public std::runtime_error
{
public:
  explicit SBPL_Exception(const std::string& what_arg = "SBPL has encountered a fatal error!")
    : std::runtime_error(what_arg)
  {
    ROS_ERROR("%s", what_arg.c_str());
  }

  explicit SBPL_Exception(const char* what_arg) : std::runtime_error(what_arg)
  {
    ROS_ERROR("%s", what_arg);
  }

  virtual ~SBPL_Exception() throw()
  {
  }
};

}  // namespace lattice_path_planner

#endif  // LATTICE_PATH_PLANNER_SBPL_EXCEPTION_H