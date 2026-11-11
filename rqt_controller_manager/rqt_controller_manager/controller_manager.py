# Copyright 2013 Georgia Tech
# Copyright 2015 PAL Robotics S.L.
# Copyright 2022 Kenji Brameld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import time

from ament_index_python.packages import get_package_share_directory
from controller_manager.controller_manager_services import ServiceNotFoundError
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import (
    CleanupController,
    ConfigureController,
    ListControllers,
    ListHardwareComponents,
    LoadController,
    SetHardwareComponentState,
    SwitchController,
    UnloadController,
)
from lifecycle_msgs.msg import State
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractTableModel, Qt, QTimer, Signal
from python_qt_binding.QtGui import QCursor, QFont, QIcon, QStandardItem, QStandardItemModel
from python_qt_binding.QtWidgets import QHeaderView, QMenu, QStyledItemDelegate, QWidget
from qt_gui.plugin import Plugin
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.duration import Duration
from ros2service.api import get_service_names_and_types

from .update_combo import update_combo


class ControllerManager(Plugin):
    """Graphical frontend for interacting with the controller manager."""

    # Emitted from the executor thread when a service call is done
    _service_done = Signal(object, object)  # future, callback

    _cm_update_freq = 1  # Hz
    # Timeout (in seconds) for the response of the periodic queries of controllers, hardware
    # components and parameters. The selected controller manager is deselected if it did not
    # respond for this long.
    _service_timeout = 2.0 / _cm_update_freq
    # Timeout (in seconds) for the response of user actions like loading or switching controllers
    _action_timeout = 10.0
    # Timeout (in seconds) for the controller manager to perform a controller switch
    _switch_timeout = 0.3

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("ControllerManager")

        # Create QWidget and extend it with all the attributes and children
        # from the UI file
        self._widget = QWidget()
        ui_file = os.path.join(
            get_package_share_directory("rqt_controller_manager"),
            "resource",
            "controller_manager.ui",
        )
        loadUi(ui_file, self._widget)
        self._widget.setObjectName("ControllerManagerUi")

        # Pop-up that displays controller information
        self._popup_widget = QWidget()
        ui_file = os.path.join(
            get_package_share_directory("rqt_controller_manager"), "resource", "popup_info.ui"
        )
        loadUi(ui_file, self._popup_widget)
        self._popup_widget.setObjectName("ControllerInfoUi")

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(f"{self._widget.windowTitle()} {context.serial_number()}")
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Initialize members
        self._cm_name = ""  # Name of the selected controller manager's node
        self._controllers = []  # State of each controller
        self._hw_components = []  # State of each hw component
        self._ctrl_table_model = None
        self._hw_table_model = None
        self._clients = {}  # service name -> client, for the selected controller manager
        self._ctrl_update_pending = False  # waiting for responses of a controllers update
        self._hw_update_pending = False  # waiting for the response of a hw components update
        self._cm_last_seen = 0.0  # time of the last response of the selected controller manager
        self._service_done.connect(self._on_service_done, Qt.ConnectionType.QueuedConnection)

        # Store reference to node
        self._node = context.node

        # Controller state icons
        path = get_package_share_directory("rqt_controller_manager")
        self._icons = {
            "active": QIcon(f"{path}/resource/led_green.png"),
            "inactive": QIcon(f"{path}/resource/led_cyan.png"),
            "unconfigured": QIcon(f"{path}/resource/led_yellow.png"),
            "unloaded": QIcon(f"{path}/resource/led_off.png"),
        }

        # Controllers display
        ctrl_table_view = self._widget.ctrl_table_view
        ctrl_table_view.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        ctrl_table_view.customContextMenuRequested.connect(self._on_ctrl_menu)
        ctrl_table_view.doubleClicked.connect(self._on_ctrl_info)

        ctrl_header = ctrl_table_view.horizontalHeader()
        ctrl_header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        ctrl_header.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        ctrl_header.customContextMenuRequested.connect(self._on_ctrl_header_menu)

        # Hardware components display
        hw_table_view = self._widget.hw_table_view
        hw_table_view.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        hw_table_view.customContextMenuRequested.connect(self._on_hw_menu)
        hw_table_view.doubleClicked.connect(self._on_hw_info)

        hw_header = hw_table_view.horizontalHeader()
        hw_header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        hw_header.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        hw_header.customContextMenuRequested.connect(self._on_hw_header_menu)

        # Timer for controller manager updates
        self._update_cm_list_timer = QTimer(self)
        self._update_cm_list_timer.setInterval(int(1000.0 / self._cm_update_freq))
        self._update_cm_list_timer.timeout.connect(self._update_cm_list)
        self._update_cm_list_timer.start()

        # Timer for running controller updates
        self._update_ctrl_list_timer = QTimer(self)
        self._update_ctrl_list_timer.setInterval(int(1000.0 / self._cm_update_freq))
        self._update_ctrl_list_timer.timeout.connect(self._update_controllers)
        self._update_ctrl_list_timer.start()

        # Timer for running hw components updates
        self._update_hw_components_list_timer = QTimer(self)
        self._update_hw_components_list_timer.setInterval(int(1000.0 / self._cm_update_freq))
        self._update_hw_components_list_timer.timeout.connect(self._update_hw_components)
        self._update_hw_components_list_timer.start()

        # Signal connections
        w = self._widget
        w.cm_combo.currentTextChanged.connect(self._on_cm_change)

    def shutdown_plugin(self):
        self._update_cm_list_timer.stop()
        self._update_ctrl_list_timer.stop()
        self._update_hw_components_list_timer.stop()
        self._destroy_clients()
        self._popup_widget.hide()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("cm_name", self._cm_name)

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore last session's controller_manager, if present
        self._update_cm_list()
        cm_name = instance_settings.value("cm_name")
        cm_combo = self._widget.cm_combo
        cm_list = [cm_combo.itemText(i) for i in range(cm_combo.count())]
        try:
            idx = cm_list.index(cm_name)
            cm_combo.setCurrentIndex(idx)
        except ValueError:
            pass

    def _update_cm_list(self):
        try:
            update_combo(self._widget.cm_combo, _list_controller_managers(self._node))
        except Exception as e:
            if "destruction was requested" in str(e) or "context is not valid" in str(e):
                self._update_cm_list_timer.stop()
                return
            raise

    def _on_cm_change(self, cm_name):
        self._cm_name = cm_name

        # Pending calls to the previously selected controller manager are dropped
        self._destroy_clients()
        self._ctrl_update_pending = False
        self._hw_update_pending = False
        # Give the service discovery time to find the newly selected controller manager
        self._cm_last_seen = time.monotonic()

        if cm_name:
            self._update_controllers()
            self._update_hw_components()
        else:
            self._controllers = []
            self._show_controllers()
            self._hw_components = []
            self._show_hw_components()

    def _deselect_cm(self, error):
        """Deselect the controller manager, e.g., because it is not reachable anymore."""
        self._node.get_logger().warning(
            f"Deselecting controller manager '{self._cm_name}': {error}"
        )
        self._widget.cm_combo.setCurrentIndex(-1)

    def _on_cm_error(self, error):
        """Deselect the controller manager if it did not respond for too long."""
        if time.monotonic() - self._cm_last_seen >= self._service_timeout:
            self._deselect_cm(error)

    def _get_client(self, srv_type, srv_name):
        client = self._clients.get(srv_name)
        if client is None:
            client = self._node.create_client(srv_type, srv_name)
            self._clients[srv_name] = client
        return client

    def _is_alive(self, client):
        return self._clients.get(client.srv_name) is client

    def _destroy_clients(self):
        for client in self._clients.values():
            self._node.destroy_client(client)
        self._clients = {}

    def _call_service(self, srv_type, srv_name, request, on_done, timeout=None):
        """
        Call a service without blocking the GUI thread.

        Calls on_done(response, error) in the GUI thread, with error being None on success. Fails
        right away if the service is not available. on_done is not called if the client was
        destroyed in the meantime, e.g., because another controller manager was selected.
        """
        if timeout is None:
            timeout = self._service_timeout
        client = self._get_client(srv_type, srv_name)
        if not client.service_is_ready():
            on_done(None, ServiceNotFoundError(f"Could not contact service {srv_name}"))
            return

        # The rqt executor spins the node, the response is handled in the GUI thread
        future = client.call_async(request)
        future.add_done_callback(lambda f: self._service_done.emit(f, on_done))
        self._start_single_shot(
            int(1000.0 * timeout), lambda: self._on_service_timeout(client, future, on_done)
        )

    def _on_service_timeout(self, client, future, on_done):
        if future.done() or not self._is_alive(client):
            return
        future.cancel()
        if not future.cancelled():
            return  # the response arrived in the meantime
        on_done(None, TimeoutError(f"Timed out waiting for a response of {client.srv_name}"))

    def _on_service_done(self, future, on_done):
        if future.cancelled():
            return  # timed out or dropped
        error = future.exception()
        on_done(None if error else future.result(), error)

    def _start_single_shot(self, msec, callback):
        # Parented to the plugin, so it does not fire after the plugin is deleted
        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.timeout.connect(callback)
        timer.timeout.connect(timer.deleteLater)
        timer.start(msec)

    def _update_controllers(self):
        """
        Request the controllers associated to the selected controller manager.

        Contains both stopped/running controllers, as returned by the C{list_controllers}
        service, plus uninitialized controllers configured in the controller manager's parameters.
        """
        if not self._cm_name or self._ctrl_update_pending:
            return

        cm_name = self._cm_name
        self._ctrl_update_pending = True
        try:
            self._call_service(
                ListControllers,
                f"{cm_name}/list_controllers",
                ListControllers.Request(),
                lambda response, error: self._on_list_controllers(cm_name, response, error),
            )
        except Exception as e:
            if "destruction was requested" in str(e) or "context is not valid" in str(e):
                self._update_ctrl_list_timer.stop()
                return
            raise

    def _on_list_controllers(self, cm_name, response, error):
        if cm_name != self._cm_name:
            return  # response for a previously selected controller manager
        if error is not None:
            self._ctrl_update_pending = False
            self._on_cm_error(error)
            return
        self._cm_last_seen = time.monotonic()

        # Append potential controller configs found in the node's parameters
        controllers = list(response.controller)
        self._call_service(
            ListParameters,
            f"{cm_name}/list_parameters",
            ListParameters.Request(),
            lambda response, error: self._on_list_parameters(
                cm_name, controllers, response, error
            ),
        )

    def _on_list_parameters(self, cm_name, controllers, response, error):
        if cm_name != self._cm_name:
            return

        suffix = ".type"
        loaded = {ctrl.name for ctrl in controllers}
        type_params = (
            []
            if error is not None
            else [
                n
                for n in response.result.names
                if n.endswith(suffix) and n[: -len(suffix)] not in loaded
            ]
        )
        if not type_params:
            self._set_controllers(controllers)
            return

        self._call_service(
            GetParameters,
            f"{cm_name}/get_parameters",
            GetParameters.Request(names=type_params),
            lambda response, error: self._on_get_controller_types(
                cm_name, controllers, type_params, response, error
            ),
        )

    def _on_get_controller_types(self, cm_name, controllers, type_params, response, error):
        if cm_name != self._cm_name:
            return

        types = [""] * len(type_params) if error is not None else response.values
        for param, type_value in zip(type_params, types):
            type_str = type_value if error is not None else type_value.string_value
            controllers.append(ControllerState(name=param[: -len(".type")], type=type_str))
        self._set_controllers(controllers)

    def _set_controllers(self, controllers):
        self._ctrl_update_pending = False

        # Update controller display, if necessary
        if self._controllers != controllers:
            self._controllers = controllers
            self._show_controllers()  # NOTE: Model is recomputed from scratch

    def _show_controllers(self):
        ctrl_table_view = self._widget.ctrl_table_view
        self._ctrl_table_model = ControllerTable(self._controllers, self._icons)
        ctrl_table_view.setModel(self._ctrl_table_model)

    def _on_ctrl_menu(self, pos):
        # Get data of selected controller
        row = self._widget.ctrl_table_view.rowAt(pos.y())
        if row < 0:
            return  # Cursor is not under a valid item

        ctrl = self._controllers[row]

        # Show context menu
        menu = QMenu(self._widget.ctrl_table_view)
        if ctrl.state == "active":
            action_deactivate = menu.addAction(self._icons["inactive"], "Deactivate (inactive)")
            action_kill = menu.addAction(
                self._icons["unloaded"], "Deactivate and Unload (unloaded)"
            )
        elif ctrl.state == "inactive":
            action_activate = menu.addAction(self._icons["active"], "Activate (active)")
            action_cleanup = menu.addAction(self._icons["unconfigured"], "Cleanup (unconfigured)")
            action_unload = menu.addAction(self._icons["unloaded"], "Unload (unloaded)")
        elif ctrl.state == "unconfigured":
            action_spawn = menu.addAction(self._icons["active"], "Configure and Activate (active)")
            action_configure = menu.addAction(self._icons["inactive"], "Configure (inactive)")
            action_unload = menu.addAction(self._icons["unloaded"], "Unload (unloaded)")
        else:
            # Controller isn't loaded
            action_load = menu.addAction(self._icons["unconfigured"], "Load (unconfigured)")

        action = menu.exec(self._widget.ctrl_table_view.mapToGlobal(pos))

        # Evaluate user action
        load = _name_step(LoadController, "load_controller", ctrl.name)
        configure = _name_step(ConfigureController, "configure_controller", ctrl.name)
        cleanup = _name_step(CleanupController, "cleanup_controller", ctrl.name)
        unload = _name_step(UnloadController, "unload_controller", ctrl.name)
        activate = self._switch_step(activate=[ctrl.name])
        deactivate = self._switch_step(deactivate=[ctrl.name])
        if ctrl.state == "active":
            if action is action_deactivate:
                self._call_steps([deactivate])
            elif action is action_kill:
                self._call_steps([deactivate, unload])
        elif ctrl.state == "inactive":
            if action is action_activate:
                self._call_steps([activate])
            elif action is action_cleanup:
                self._call_steps([cleanup])
            elif action is action_unload:
                self._call_steps([unload])
        elif ctrl.state == "unconfigured":
            if action is action_configure:
                self._call_steps([configure])
            elif action is action_spawn:
                self._call_steps([configure, activate])
            elif action is action_unload:
                self._call_steps([unload])
        else:
            # Assume controller isn't loaded
            if action is action_load:
                self._call_steps([load])

    def _on_ctrl_info(self, index):
        popup = self._popup_widget
        popup.setWindowTitle("Controller Information")

        # Reset fields to prevent stale data when switching between popups
        popup.ctrl_update_rate.setText("N/A")
        popup.ctrl_is_async.setText("N/A")

        ctrl = self._controllers[index.row()]
        popup.ctrl_name.setText(ctrl.name)
        popup.ctrl_type.setText(ctrl.type)
        popup.ctrl_update_rate.setText(f"{ctrl.update_rate} Hz")
        popup.ctrl_is_async.setText(str(ctrl.is_async))

        res_model = QStandardItemModel()
        model_root = QStandardItem("Claimed Interfaces")
        res_model.appendRow(model_root)
        for claimed_interface in ctrl.claimed_interfaces:
            hw_iface_item = QStandardItem(claimed_interface)
            model_root.appendRow(hw_iface_item)

        popup.resource_tree.setModel(res_model)
        popup.resource_tree.setItemDelegate(FontDelegate(popup.resource_tree))
        popup.resource_tree.expandAll()
        popup.move(QCursor.pos())
        popup.show()

    def _on_ctrl_header_menu(self, pos):
        ctrl_header = self._widget.ctrl_table_view.horizontalHeader()

        # Show context menu
        menu = QMenu(self._widget.ctrl_table_view)
        action_toggle_auto_resize = menu.addAction("Toggle Auto-Resize")
        action = menu.exec(ctrl_header.mapToGlobal(pos))

        # Evaluate user action
        if action is action_toggle_auto_resize:
            if ctrl_header.sectionResizeMode(0) == QHeaderView.ResizeMode.ResizeToContents:
                ctrl_header.setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
            else:
                ctrl_header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)

    def _update_hw_components(self):
        """Request the hw_components associated to the selected controller manager."""
        if not self._cm_name or self._hw_update_pending:
            return

        cm_name = self._cm_name
        self._hw_update_pending = True
        try:
            self._call_service(
                ListHardwareComponents,
                f"{cm_name}/list_hardware_components",
                ListHardwareComponents.Request(),
                lambda response, error: self._on_list_hw_components(cm_name, response, error),
            )
        except Exception as e:
            if "destruction was requested" in str(e) or "context is not valid" in str(e):
                self._update_hw_components_list_timer.stop()
                return
            raise

    def _on_list_hw_components(self, cm_name, response, error):
        if cm_name != self._cm_name:
            return  # response for a previously selected controller manager
        self._hw_update_pending = False
        if error is not None:
            self._on_cm_error(error)
            return
        self._cm_last_seen = time.monotonic()

        # Update hw components display, if necessary
        hw_components = list(response.component)
        if self._hw_components != hw_components:
            self._hw_components = hw_components
            self._show_hw_components()  # NOTE: Model is recomputed from scratch

    def _show_hw_components(self):
        hw_table_view = self._widget.hw_table_view
        self._hw_table_model = HwComponentTable(self._hw_components, self._icons)
        hw_table_view.setModel(self._hw_table_model)

    def _on_hw_menu(self, pos):
        # Get data of selected controller
        row = self._widget.hw_table_view.rowAt(pos.y())
        if row < 0:
            return  # Cursor is not under a valid item

        hw_component = self._hw_components[row]

        # Show context menu
        menu = QMenu(self._widget.hw_table_view)
        if hw_component.state.label == "active":
            action_deactivate = menu.addAction(self._icons["inactive"], "Deactivate")
            action_cleanup = menu.addAction(self._icons["unconfigured"], "Deactivate and Cleanup")
        elif hw_component.state.label == "inactive":
            action_activate = menu.addAction(self._icons["active"], "Activate")
            action_cleanup = menu.addAction(self._icons["unconfigured"], "Cleanup")
        elif hw_component.state.label == "unconfigured":
            action_configure = menu.addAction(self._icons["inactive"], "Configure")
            action_spawn = menu.addAction(self._icons["active"], "Configure and Activate")

        action = menu.exec(self._widget.hw_table_view.mapToGlobal(pos))

        # Evaluate user action
        if hw_component.state.label == "active":
            if action is action_deactivate:
                self._set_inactive_hw_component(hw_component.name)
            elif action is action_cleanup:
                self._set_unconfigured_hw_component(hw_component.name)
        elif hw_component.state.label == "inactive":
            if action is action_activate:
                self._set_active_hw_component(hw_component.name)
            elif action is action_cleanup:
                self._set_unconfigured_hw_component(hw_component.name)
        elif hw_component.state.label == "unconfigured":
            if action is action_configure:
                self._set_inactive_hw_component(hw_component.name)
            elif action is action_spawn:
                self._set_active_hw_component(hw_component.name)

    def _on_hw_info(self, index):
        popup = self._popup_widget
        popup.setWindowTitle("Hardware Component Info")

        # Reset fields to prevent stale data when switching between popups
        popup.ctrl_update_rate.setText("N/A")
        popup.ctrl_is_async.setText("N/A")

        hw_component = self._hw_components[index.row()]
        popup.ctrl_name.setText(hw_component.name)
        popup.ctrl_type.setText(hw_component.type)
        popup.ctrl_update_rate.setText(f"{hw_component.rw_rate} Hz")
        popup.ctrl_is_async.setText(str(hw_component.is_async))

        res_model = QStandardItemModel()
        model_root = QStandardItem("Command Interfaces")
        res_model.appendRow(model_root)
        for command_interface in hw_component.command_interfaces:
            hw_iface_item = QStandardItem(command_interface.name)
            model_root.appendRow(hw_iface_item)

        model_root = QStandardItem("State Interfaces")
        res_model.appendRow(model_root)
        for state_interface in hw_component.state_interfaces:
            hw_iface_item = QStandardItem(state_interface.name)
            model_root.appendRow(hw_iface_item)

        popup.resource_tree.setModel(res_model)
        popup.resource_tree.setItemDelegate(FontDelegate(popup.resource_tree))
        popup.resource_tree.expandAll()
        popup.move(QCursor.pos())
        popup.show()

    def _on_hw_header_menu(self, pos):
        hw_header = self._widget.hw_table_view.horizontalHeader()

        # Show context menu
        menu = QMenu(self._widget.hw_table_view)
        action_toggle_auto_resize = menu.addAction("Toggle Auto-Resize")
        action = menu.exec(hw_header.mapToGlobal(pos))

        # Evaluate user action
        if action is action_toggle_auto_resize:
            if hw_header.sectionResizeMode(0) == QHeaderView.ResizeMode.ResizeToContents:
                hw_header.setSectionResizeMode(QHeaderView.ResizeMode.Interactive)
            else:
                hw_header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)

    def _switch_step(self, activate=(), deactivate=()):
        request = SwitchController.Request(
            activate_controllers=list(activate),
            deactivate_controllers=list(deactivate),
            strictness=SwitchController.Request.STRICT,
            activate_asap=False,
            timeout=Duration(seconds=self._switch_timeout).to_msg(),
        )
        return SwitchController, "switch_controller", request

    def _call_steps(self, steps):
        """
        Call the controller manager services of the steps in order, stopping at the first failure.

        @param steps List of (service type, service name relative to the controller manager,
        request)
        """
        if not steps:
            # Show the outcome right away instead of waiting for the next periodic update
            self._update_controllers()
            self._update_hw_components()
            return

        cm_name = self._cm_name
        srv_type, service, request = steps[0]
        self._call_service(
            srv_type,
            f"{cm_name}/{service}",
            request,
            lambda response, error: self._on_step_done(
                cm_name, service, steps[1:], response, error
            ),
            self._action_timeout,
        )

    def _on_step_done(self, cm_name, service, steps, response, error):
        if cm_name != self._cm_name:
            return
        if error is None and not response.ok:
            error = RuntimeError("the controller manager reported a failure")
        if error is not None:
            self._node.get_logger().error(f"Calling {cm_name}/{service} failed: {error}")
            return
        self._call_steps(steps)

    def _set_active_hw_component(self, name):
        active_state = State()
        active_state.id = State.PRIMARY_STATE_ACTIVE
        active_state.label = "active"
        self._set_state_hw_component(name, active_state)

    def _set_inactive_hw_component(self, name):
        inactive_state = State()
        inactive_state.id = State.PRIMARY_STATE_INACTIVE
        inactive_state.label = "inactive"
        self._set_state_hw_component(name, inactive_state)

    def _set_unconfigured_hw_component(self, name):
        unconfigure_state = State()
        unconfigure_state.id = State.PRIMARY_STATE_UNCONFIGURED
        unconfigure_state.label = "unconfigured"
        self._set_state_hw_component(name, unconfigure_state)

    def _set_state_hw_component(self, name, state):
        request = SetHardwareComponentState.Request(name=name, target_state=state)
        self._call_steps([(SetHardwareComponentState, "set_hardware_component_state", request)])


class ControllerTable(QAbstractTableModel):
    """
    Model containing controller information for tabular display.

    The model allows display of basic read-only information like controller
    name and state.
    """

    def __init__(self, controller_info, icons, parent=None):
        QAbstractTableModel.__init__(self, parent)
        self._data = controller_info
        self._icons = icons

    def rowCount(self, parent):
        return len(self._data)

    def columnCount(self, parent):
        return 2

    def headerData(self, col, orientation, role):
        if orientation != Qt.Orientation.Horizontal or role != Qt.ItemDataRole.DisplayRole:
            return None
        if col == 0:
            return "controller"
        elif col == 1:
            return "state"

    def data(self, index, role):
        if not index.isValid():
            return None

        ctrl = self._data[index.row()]

        if role == Qt.ItemDataRole.DisplayRole:
            if index.column() == 0:
                return ctrl.name
            elif index.column() == 1:
                return ctrl.state or "unloaded"

        if role == Qt.ItemDataRole.DecorationRole and index.column() == 0:
            state_key = ctrl.state if ctrl.state else "unloaded"
            return self._icons.get(state_key)

        if role == Qt.ItemDataRole.FontRole and index.column() == 0:
            bf = QFont()
            bf.setBold(True)
            return bf

        if role == Qt.ItemDataRole.TextAlignmentRole and index.column() == 1:
            return Qt.AlignmentFlag.AlignCenter


class HwComponentTable(QAbstractTableModel):
    """
    Model containing hardware component information for tabular display.

    The model allows display of basic read-only information like component
    name and state.
    """

    def __init__(self, hw_component_info, icons, parent=None):
        QAbstractTableModel.__init__(self, parent)
        self._data = hw_component_info
        self._icons = icons

    def rowCount(self, parent):
        return len(self._data)

    def columnCount(self, parent):
        return 2

    def headerData(self, col, orientation, role):
        if orientation != Qt.Orientation.Horizontal or role != Qt.ItemDataRole.DisplayRole:
            return None
        if col == 0:
            return "component"
        elif col == 1:
            return "state"

    def data(self, index, role):
        if not index.isValid():
            return None

        hw_component = self._data[index.row()]

        if role == Qt.ItemDataRole.DisplayRole:
            if index.column() == 0:
                return hw_component.name
            elif index.column() == 1:
                return hw_component.state.label or "unloaded"

        if role == Qt.ItemDataRole.DecorationRole and index.column() == 0:
            return self._icons.get(hw_component.state.label)

        if role == Qt.ItemDataRole.FontRole and index.column() == 0:
            bf = QFont()
            bf.setBold(True)
            return bf

        if role == Qt.ItemDataRole.TextAlignmentRole and index.column() == 1:
            return Qt.AlignmentFlag.AlignCenter


class FontDelegate(QStyledItemDelegate):
    """
    Simple delegate for customizing font weight and italization.

    Simple delegate for customizing font weight and italization when displaying resources claimed
    by a controller.
    """

    def paint(self, painter, option, index):
        if not index.parent().isValid():
            # Root level
            option.font.setWeight(QFont.Weight.Bold)
        if index.parent().isValid() and not index.parent().parent().isValid():
            # Hardware interface level
            option.font.setItalic(True)
            option.font.setWeight(QFont.Weight.Bold)
        QStyledItemDelegate.paint(self, painter, option, index)


def _list_controller_managers(node):
    """
    List controller manager nodes that are active.

    Does this by looking for a service that should be exclusive to a controller manager node.
    The "list_controllers" service is used to determine if a node is a controller manager.
    @return List of controller manager node names
    @rtype list of str
    """
    try:
        return [
            name.rstrip("list_controllers").rstrip("/")
            for name, _ in get_service_names_and_types(node=node)
            if name.endswith("list_controllers")
        ]
    except Exception as e:
        if "destruction was requested" in str(e) or "context is not valid" in str(e):
            return []
        raise


def _name_step(srv_type, service, name):
    """Step for ControllerManager._call_steps with a request that only takes a name."""
    return srv_type, service, srv_type.Request(name=name)
