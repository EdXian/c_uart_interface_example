#include <common/mavlink.h>

#include "port_setup.h"

struct timeval tv;		  ///< System time

// Settings
int sysid = 1;             ///< The unique system id of this MAV, 0-127. Has to be consistent across the system
int compid = 50;
int serial_compid = 0;			//Code for all
bool silent = false;              ///< Wether console output should be enabled
bool verbose = false;             ///< Enable verbose output
bool debug = false;               ///< Enable debug functions and output
int fd;
