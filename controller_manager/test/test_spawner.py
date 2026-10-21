# Copyright 2026 The ros2_control Development Team
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

import signal

from controller_manager.spawner import _on_shutdown_signal
import pytest


def test_unload_on_kill_ignores_signals_before_raising(monkeypatch):
    registered_signals = []
    shutdown_state = {'started': False}

    def shutdown_signal_handler(signum, frame):
        _on_shutdown_signal(
            signum, frame, unload_on_kill=True, shutdown_state=shutdown_state
        )

    current_handlers = {
        signal.SIGINT: shutdown_signal_handler,
        signal.SIGTERM: shutdown_signal_handler,
    }

    def register_signal(signum, handler):
        registered_signals.append((signum, handler))
        if handler is signal.SIG_IGN and current_handlers[signum] is shutdown_signal_handler:
            current_handlers[signum](signum, None)
        current_handlers[signum] = handler

    monkeypatch.setattr(signal, 'signal', register_signal)

    with pytest.raises(KeyboardInterrupt):
        _on_shutdown_signal(
            signal.SIGINT, None, unload_on_kill=True, shutdown_state=shutdown_state
        )

    assert registered_signals == [
        (signal.SIGINT, signal.SIG_IGN),
        (signal.SIGTERM, signal.SIG_IGN),
    ]

    _on_shutdown_signal(signal.SIGTERM, None, unload_on_kill=True, shutdown_state=shutdown_state)
    assert registered_signals == [
        (signal.SIGINT, signal.SIG_IGN),
        (signal.SIGTERM, signal.SIG_IGN),
    ]


def test_regular_spawner_does_not_ignore_signals_in_handler(monkeypatch):
    monkeypatch.setattr(
        signal,
        'signal',
        lambda signum, handler: pytest.fail('handler must not ignore signals'),
    )

    with pytest.raises(KeyboardInterrupt):
        _on_shutdown_signal(
            signal.SIGINT,
            None,
            unload_on_kill=False,
            shutdown_state={'started': False},
        )
