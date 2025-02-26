# This message is used to provide the activity within the controller manager regarding the change in state of controllers and hardware interfaces

# The header is used to provide timestamp information
std_msgs/Header header

# The current state of the controllers
NamedLifecycleState[] controllers

# The current state of the hardware components
NamedLifecycleState[] hardware_components
