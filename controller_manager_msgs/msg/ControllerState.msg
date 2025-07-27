string name        # controller name
string state        # controller state: unconfigured, inactive, active, or finalized
string type        # the controller class name, e.g. joint_trajectory_controller/JointTrajectoryController
bool is_async       # true, if controller runs asynchronously. false, if controller runs synchronously
uint16 update_rate        # update rate of the controller in Hz
string[] claimed_interfaces        # command interfaces currently owned by controller
string[] required_command_interfaces        # command interfaces required by controller
string[] required_state_interfaces        # state interfaces required by controller
bool is_chainable        # specifies whether or not controller can export references for a controller chain
bool is_chained        # specifies whether or not controller's exported references are claimed by another controller
string[] exported_state_interfaces        # state interfaces to be exported (only applicable if is_chainable is true)
string[] reference_interfaces        # references to be exported (only applicable if is_chainable is true)
ChainConnection[] chain_connections        # specifies list of controllers and their exported references that the controller is chained to
