# Copyright (c) 2024 AIT - Austrian Institute of Technology GmbH
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Christoph Froehlich

import time

from launch_testing_ros import WaitForTopics
from sensor_msgs.msg import JointState
from controller_manager.controller_manager_services import list_controllers


def check_node_running(node, node_name, timeout=5.0):

    start = time.time()
    found = False
    while time.time() - start < timeout and not found:
        found = node_name in node.get_node_names()
        time.sleep(0.1)
    assert found, f"{node_name} not found!"


def check_controllers_running(node, cnames, namespace="", state="active"):

    # wait for controller to be loaded before we call the CM services
    found = {cname: False for cname in cnames}  # Define 'found' as a dictionary
    start = time.time()
    # namespace is either "/" (empty) or "/ns" if set
    if namespace:
        namespace_api = namespace
        if not namespace_api.startswith("/"):
            namespace_api = "/" + namespace_api
        if namespace.endswith("/"):
            namespace_api = namespace_api[:-1]
    else:
        namespace_api = "/"

    while time.time() - start < 10.0 and not all(found.values()):
        node_names_namespaces = node.get_node_names_and_namespaces()
        for cname in cnames:
            if any(name == cname and ns == namespace_api for name, ns in node_names_namespaces):
                found[cname] = True
        time.sleep(0.1)
    assert all(
        found.values()
    ), f"Controller node(s) not found: {', '.join(['ns: ' + namespace_api + ', ctrl:' + cname for cname, is_found in found.items() if not is_found])}, but seeing {node.get_node_names_and_namespaces()}"

    found = {cname: False for cname in cnames}  # Define 'found' as a dictionary
    start = time.time()
    # namespace is either "/" (empty) or "/ns" if set
    if not namespace:
        cm = "controller_manager"
    else:
        if namespace.endswith("/"):
            cm = namespace + "controller_manager"
        else:
            cm = namespace + "/controller_manager"
    while time.time() - start < 10.0 and not all(found.values()):
        controllers = list_controllers(node, cm, 5.0).controller
        assert controllers, "No controllers found!"
        for c in controllers:
            for cname in cnames:
                if c.name == cname and c.state == state:
                    found[cname] = True
                    break
        time.sleep(0.1)

    assert all(
        found.values()
    ), f"Controller(s) not found or not {state}: {', '.join([cname for cname, is_found in found.items() if not is_found])}"


def check_if_js_published(topic, joint_names):
    wait_for_topics = WaitForTopics([(topic, JointState)], timeout=20.0)
    assert wait_for_topics.wait(), f"Topic '{topic}' not found!"
    msgs = wait_for_topics.received_messages(topic)
    msg = msgs[0]
    assert len(msg.name) == len(joint_names), "Wrong number of joints in message"
    # use a set to compare the joint names, as the order might be different
    assert set(msg.name) == set(joint_names), "Wrong joint names"
    wait_for_topics.shutdown()
