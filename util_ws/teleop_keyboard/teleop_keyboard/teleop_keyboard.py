#!/usr/bin/env python3
"""
teleop_keyboard.py

ROS2 port of teleop_keyboard.py:
Translates keyboard input to velocity commands for the JetRacer.

Created by Magnus Mortensen using perplexity.ai with the Gemini 3.1 pro model.
Using the ROS 1 teleop_keyboard.py as a source. The script was retrieved from the original ROS 1 jetson Nano Image.
"""

import ctypes
import ctypes.util
import platform
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

INSTRUCTIONS = """
Keyboard teleop for JetRacer
----------------------------
Hold W/S : forward/backward
Hold A/D : steer left/right
X/Space  : stop
Q        : quit

On macOS this node uses a global keyboard event tap.
On Linux this node uses X11 keyboard state polling.
""".strip()

MAC_KEY_CODES = {
    13: 'w',
    1: 's',
    0: 'a',
    2: 'd',
    7: 'x',
    49: ' ',
    12: 'q',
}


class MacKeyListener:
    KCG_EVENT_KEY_DOWN = 10
    KCG_EVENT_KEY_UP = 11
    KCG_SESSION_EVENT_TAP = 1
    KCG_HEAD_INSERT_EVENT_TAP = 0
    KCG_EVENT_TAP_OPTION_DEFAULT = 0
    KCG_KEYBOARD_EVENT_KEYCODE = 9

    def __init__(self, on_key_event):
        self.on_key_event = on_key_event
        self._thread = None
        self._tap = None
        self._run_loop = None
        self._source = None
        self._callback = None
        self._startup_error = None

        self._application_services = ctypes.cdll.LoadLibrary(
            '/System/Library/Frameworks/ApplicationServices.framework/ApplicationServices'
        )
        self._core_foundation = ctypes.cdll.LoadLibrary(
            '/System/Library/Frameworks/CoreFoundation.framework/CoreFoundation'
        )

        self._CFMachPortRef = ctypes.c_void_p
        self._CFRunLoopRef = ctypes.c_void_p
        self._CFRunLoopSourceRef = ctypes.c_void_p
        self._CGEventRef = ctypes.c_void_p
        self._CGEventTapProxy = ctypes.c_void_p
        self._CGEventMask = ctypes.c_uint64
        self._CFAllocatorRef = ctypes.c_void_p
        self._CFIndex = ctypes.c_long

        self._event_callback_type = ctypes.CFUNCTYPE(
            ctypes.c_void_p,
            self._CGEventTapProxy,
            ctypes.c_uint32,
            self._CGEventRef,
            ctypes.c_void_p,
        )

        self._bind_functions()

    def _bind_functions(self):
        app = self._application_services
        cf = self._core_foundation

        app.CGEventTapCreate.argtypes = [
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_uint32,
            self._CGEventMask,
            self._event_callback_type,
            ctypes.c_void_p,
        ]
        app.CGEventTapCreate.restype = self._CFMachPortRef

        app.CGEventTapEnable.argtypes = [self._CFMachPortRef, ctypes.c_bool]
        app.CGEventTapEnable.restype = None

        app.CGEventGetIntegerValueField.argtypes = [self._CGEventRef, ctypes.c_int32]
        app.CGEventGetIntegerValueField.restype = ctypes.c_int64

        cf.CFMachPortCreateRunLoopSource.argtypes = [
            self._CFAllocatorRef,
            self._CFMachPortRef,
            self._CFIndex,
        ]
        cf.CFMachPortCreateRunLoopSource.restype = self._CFRunLoopSourceRef

        cf.CFRunLoopGetCurrent.argtypes = []
        cf.CFRunLoopGetCurrent.restype = self._CFRunLoopRef

        cf.CFRunLoopAddSource.argtypes = [
            self._CFRunLoopRef,
            self._CFRunLoopSourceRef,
            ctypes.c_void_p,
        ]
        cf.CFRunLoopAddSource.restype = None

        cf.CFRunLoopRun.argtypes = []
        cf.CFRunLoopRun.restype = ctypes.c_int32

        cf.CFRunLoopStop.argtypes = [self._CFRunLoopRef]
        cf.CFRunLoopStop.restype = None

        cf.CFRelease.argtypes = [ctypes.c_void_p]
        cf.CFRelease.restype = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._thread.join(timeout=0.2)
        if self._startup_error is not None:
            raise RuntimeError(self._startup_error)

    def stop(self):
        if self._run_loop:
            self._core_foundation.CFRunLoopStop(self._run_loop)
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

        if self._source:
            self._core_foundation.CFRelease(self._source)
            self._source = None
        if self._tap:
            self._core_foundation.CFRelease(self._tap)
            self._tap = None

    def _run(self):
        def callback(_proxy, event_type, event, _refcon):
            if event_type in (self.KCG_EVENT_KEY_DOWN, self.KCG_EVENT_KEY_UP):
                key_code = self._application_services.CGEventGetIntegerValueField(
                    event, self.KCG_KEYBOARD_EVENT_KEYCODE
                )
                key = MAC_KEY_CODES.get(key_code)
                if key is not None:
                    self.on_key_event(key, event_type == self.KCG_EVENT_KEY_DOWN)
            return event

        self._callback = self._event_callback_type(callback)
        mask = (
                (1 << self.KCG_EVENT_KEY_DOWN)
                | (1 << self.KCG_EVENT_KEY_UP)
        )

        self._tap = self._application_services.CGEventTapCreate(
            self.KCG_SESSION_EVENT_TAP,
            self.KCG_HEAD_INSERT_EVENT_TAP,
            self.KCG_EVENT_TAP_OPTION_DEFAULT,
            mask,
            self._callback,
            None,
        )
        if not self._tap:
            self._startup_error = (
                'Failed to create macOS keyboard event tap. '
                'Grant Terminal accessibility/input monitoring permission.'
            )
            return

        common_modes = ctypes.c_void_p.in_dll(
            self._core_foundation, 'kCFRunLoopCommonModes'
        )
        self._source = self._core_foundation.CFMachPortCreateRunLoopSource(
            None, self._tap, 0
        )
        self._run_loop = self._core_foundation.CFRunLoopGetCurrent()
        self._core_foundation.CFRunLoopAddSource(
            self._run_loop, self._source, common_modes
        )
        self._application_services.CGEventTapEnable(self._tap, True)
        self._core_foundation.CFRunLoopRun()


class LinuxKeyListener:
    def __init__(self, on_key_event):
        self.on_key_event = on_key_event
        self._thread = None
        self._running = False
        self._display = None
        self._startup_error = None
        self._pressed_keys = set()

        x11_path = ctypes.util.find_library('X11')
        if not x11_path:
            raise RuntimeError('libX11 not found. Linux keyboard backend requires X11.')
        self._x11 = ctypes.cdll.LoadLibrary(x11_path)
        self._bind_functions()

    def _bind_functions(self):
        self._x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self._x11.XOpenDisplay.restype = ctypes.c_void_p

        self._x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self._x11.XCloseDisplay.restype = ctypes.c_int

        self._x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
        self._x11.XStringToKeysym.restype = ctypes.c_ulong

        self._x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self._x11.XKeysymToKeycode.restype = ctypes.c_uint

        self._x11.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_char * 32]
        self._x11.XQueryKeymap.restype = ctypes.c_int

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._thread.join(timeout=0.2)
        if self._startup_error is not None:
            raise RuntimeError(self._startup_error)

    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._display:
            self._x11.XCloseDisplay(self._display)
            self._display = None

    def _run(self):
        self._display = self._x11.XOpenDisplay(None)
        if not self._display:
            self._startup_error = (
                'Failed to open X11 display. Linux keyboard backend requires an X11 session.'
            )
            return

        watched_keys = ('w', 'a', 's', 'd', 'x', ' ', 'q')
        keycodes = {}
        for key in watched_keys:
            keysym_name = b'space' if key == ' ' else key.encode('ascii')
            keysym = self._x11.XStringToKeysym(keysym_name)
            keycode = self._x11.XKeysymToKeycode(self._display, keysym)
            if keycode:
                keycodes[key] = int(keycode)

        self._running = True
        while self._running:
            keymap = (ctypes.c_char * 32)()
            self._x11.XQueryKeymap(self._display, keymap)
            currently_pressed = {
                key for key, keycode in keycodes.items()
                if self._is_key_pressed(keymap, keycode)
            }

            for key in currently_pressed - self._pressed_keys:
                self.on_key_event(key, True)
            for key in self._pressed_keys - currently_pressed:
                self.on_key_event(key, False)

            self._pressed_keys = currently_pressed
            time.sleep(0.01)

    @staticmethod
    def _is_key_pressed(keymap, keycode: int) -> bool:
        index = keycode // 8
        bit = keycode % 8
        value = keymap[index]
        if isinstance(value, bytes):
            value = value[0]
        return bool(value & (1 << bit))


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.6)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('publish_rate', 20.0)

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.cmd = Twist()
        self._pressed_keys = set()
        self._quit_requested = False

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.05
        self.timer = self.create_timer(period, self.publish_command)

    def publish_command(self):
        self.cmd_pub.publish(self.cmd)

    def handle_key_event(self, key: str, pressed: bool):
        if key == 'q' and pressed:
            self._quit_requested = True
            self._pressed_keys.clear()
            self.cmd = Twist()
            self.cmd_pub.publish(self.cmd)
            return

        if key in ('x', ' ') and pressed:
            self._pressed_keys.clear()
            self.cmd = Twist()
            self.cmd_pub.publish(self.cmd)
            return

        if key not in ('w', 'a', 's', 'd'):
            return

        if pressed:
            self._pressed_keys.add(key)
        else:
            self._pressed_keys.discard(key)

        self._update_command_from_state()
        self.cmd_pub.publish(self.cmd)

    def should_quit(self) -> bool:
        return self._quit_requested

    def _update_command_from_state(self):
        forward = 'w' in self._pressed_keys
        reverse = 's' in self._pressed_keys
        left = 'a' in self._pressed_keys
        right = 'd' in self._pressed_keys

        if forward == reverse:
            self.cmd.linear.x = 0.0
        elif forward:
            self.cmd.linear.x = self.linear_speed
        else:
            self.cmd.linear.x = -self.linear_speed

        if left == right:
            self.cmd.angular.z = 0.0
        elif left:
            self.cmd.angular.z = self.angular_speed
        else:
            self.cmd.angular.z = -self.angular_speed


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    listener = None

    print(INSTRUCTIONS)

    try:
        current_platform = platform.system()
        if current_platform == 'Darwin':
            listener = MacKeyListener(node.handle_key_event)
        elif current_platform == 'Linux':
            listener = LinuxKeyListener(node.handle_key_event)
        else:
            raise RuntimeError(
                'Key up/down event handling is implemented only for macOS and Linux/X11.'
            )
        listener.start()

        while rclpy.ok() and not node.should_quit():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.cmd = Twist()
        node.publish_command()
        if listener is not None:
            listener.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
