string name # Name of the hardware component
string type # Type of the hardware component
bool is_async # If the hardware component is running asynchronously
uint16 rw_rate # read/write rate of the hardware component in Hz
string class_type  # DEPRECATED
string plugin_name # The name of the plugin that is used to load the hardware component
lifecycle_msgs/State state # State of the hardware component
HardwareInterface[] command_interfaces # Command interfaces of the hardware component
HardwareInterface[] state_interfaces # State interfaces of the hardware component
