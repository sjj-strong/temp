"""Async controller_manager client — never spins the executor (spec §5, 问题 10)."""

import logging

from controller_manager_msgs.srv import (
    SwitchController,
    ListControllers,
    LoadController,
)

logger = logging.getLogger(__name__)


class ControllerSwitcher:
    """Client for /controller_manager services.

    All calls are async (call_async + returned Future); the caller polls
    future.done() from its own state machine — the switcher never blocks.
    """

    def __init__(self, node, timeout_s: float = 10.0):
        self._node = node
        self._timeout_s = timeout_s
        self._switch_cli = node.create_client(SwitchController, "/controller_manager/switch_controller")
        self._list_cli = node.create_client(ListControllers, "/controller_manager/list_controllers")
        self._load_cli = node.create_client(LoadController, "/controller_manager/load_controller")

    def services_ready(self) -> bool:
        return (self._switch_cli.service_is_ready()
                and self._list_cli.service_is_ready()
                and self._load_cli.service_is_ready())

    def wait_for_services(self, timeout_s: float | None = None) -> bool:
        """Blocking wait (main thread / one-shot processes only)."""
        timeout = timeout_s if timeout_s is not None else self._timeout_s
        deadline = self._node.get_clock().now().nanoseconds / 1e9 + timeout
        for cli, name in [
            (self._load_cli, "load_controller"),
            (self._switch_cli, "switch_controller"),
            (self._list_cli, "list_controllers"),
        ]:
            while self._node.get_clock().now().nanoseconds / 1e9 < deadline and self._node.context.ok():
                if cli.wait_for_service(timeout_sec=0.5):
                    break
            else:
                logger.error(f"Timed out waiting for /controller_manager/{name}")
                return False
        return True

    def list_controllers(self):
        """Future resolving to the ListControllers response; None if client not ready."""
        if not self._list_cli.service_is_ready():
            return None
        return self._list_cli.call_async(ListControllers.Request())

    @staticmethod
    def list_result(future) -> dict:
        if future is None or not future.done() or future.result() is None:
            return {}
        return {c.name: c.state for c in future.result().controller}

    def load_controller(self, controller_name: str):
        """Future resolving to a LoadController response (result.ok on success)."""
        if not self._load_cli.service_is_ready():
            return None
        req = LoadController.Request()
        req.name = controller_name
        return self._load_cli.call_async(req)

    def switch(self, activate: list[str], deactivate: list[str]):
        """Strict switch request; returns Future, or None if client not ready."""
        if not self._switch_cli.service_is_ready():
            return None
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.STRICT
        return self._switch_cli.call_async(req)

    @staticmethod
    def switch_ok(future) -> bool:
        return (
            future is not None
            and future.done()
            and future.result() is not None
            and future.result().ok
        )
