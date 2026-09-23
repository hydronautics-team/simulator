#!/usr/bin/env python3
"""Live matplotlib plots for robot debugging (started only with debug:=true).

One window per perspective; every entry of the perspective `topics` list gets
its own subplot (in one or two columns, shared time axis). The topics are
subscribed to lazily: the node waits for a topic to appear, looks up its
message type and then subscribes, so no message types are hardcoded here and
nested fields are supported (e.g. /<name>/debug/pose/pose/position/z).

The window is scrollable: the figure is embedded in a Tk canvas with a
vertical scrollbar (mouse wheel and Page Up/Down work, `q` closes the window),
so tall perspectives like odometry with all IMU channels stay readable.

Parameters (set by the spawn launch from descriptions/config/plots/<name>.yaml):

  topics          list of 'topic/field' entries, one subplot each
  window_title    window caption
  window_seconds  time span shown on the plots (default 60)
  update_rate     redraw rate in Hz (default 10)

The TkAgg backend is used so the windows go through X11; when Tk is not
available (headless) the node falls back to Agg and just keeps gathering data.
"""

import os
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.utilities import get_message

import matplotlib
# Honour an explicit MPLBACKEND (tests/headless use Agg); otherwise prefer the
# Tk backend so the windows go through X11.
if not os.environ.get('MPLBACKEND'):
    try:
        matplotlib.use('TkAgg')
    except Exception:
        matplotlib.use('Agg')
from matplotlib.animation import FuncAnimation
from matplotlib.figure import Figure

if matplotlib.get_backend().lower().startswith('tk'):
    import tkinter as tk
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


def _resolve_topic(spec, topic_names):
    """Split a 'topic/field' spec using the known topics.

    The topic is the longest prefix of the spec that exists as a topic; the
    rest of the path is the field inside the message.
    """
    parts = spec.split('/')
    for i in range(len(parts) - 1, 0, -1):
        candidate = '/'.join(parts[:i])
        if candidate in topic_names:
            return candidate, [p for p in parts[i:] if p]
    return None, None


def _field_value(message, field_parts):
    """Numeric value of a nested message field, or None."""
    value = message
    for part in field_parts:
        try:
            value = getattr(value, part)
        except AttributeError:
            return None
    if isinstance(value, bool):
        return float(value)
    if isinstance(value, (int, float)):
        return float(value)
    return None


class DebugPlot(Node):
    def __init__(self):
        super().__init__('debug_plot')

        self.declare_parameter('topics', [''])
        self.declare_parameter('window_title', 'debug')
        self.declare_parameter('window_seconds', 60.0)
        self.declare_parameter('update_rate', 10.0)

        specs = [s for s in self.get_parameter('topics').value if s]
        self.window_seconds = max(
            1.0, float(self.get_parameter('window_seconds').value))
        self.update_rate = max(
            1.0, float(self.get_parameter('update_rate').value))
        title = self.get_parameter('window_title').value

        self.start_time = time.monotonic()
        self.lock = threading.Lock()

        # One curve per spec; 'topic' and 'field' are resolved lazily.
        self.curves = []
        for spec in specs:
            self.curves.append({
                'spec': spec,
                'topic': None,
                'field': None,
                'times': deque(),
                'values': deque(),
            })
        self.pending = list(self.curves)
        # topic -> list of curves fed by that topic
        self.attached = {}
        self.topic_subscriptions = {}
        # topic -> number of publishers reported last time
        self.publisher_counts = {}

        self._build_figure(title)

        # Poll for the topics to appear and subscribe to them.
        self.create_timer(1.0, self._try_subscribe)

    # -- subscription ------------------------------------------------------

    def _try_subscribe(self):
        self._check_publishers()
        if not self.pending:
            return

        types_and_names = self.get_topic_names_and_types()
        topics = {name: types[0] for name, types in types_and_names}
        remaining = []
        for curve in self.pending:
            topic = curve['topic']
            if topic is None:
                topic, field = _resolve_topic(curve['spec'], topics.keys())
                if topic is None:
                    remaining.append(curve)
                    continue
                curve['topic'] = topic
                curve['field'] = field

            if topic in self.topic_subscriptions:
                self.attached.setdefault(topic, []).append(curve)
                continue
            if topic not in topics:
                remaining.append(curve)
                continue
            try:
                message_type = get_message(topics[topic])
            except Exception as error:  # noqa: BLE001
                self.get_logger().warning(
                    'cannot load message type of %s: %s' % (topic, error))
                remaining.append(curve)
                continue

            self.topic_subscriptions[topic] = self.create_subscription(
                message_type, topic,
                lambda message, t=topic: self._on_message(t, message), 10)
            self.attached.setdefault(topic, []).append(curve)
            self.get_logger().info('subscribed to %s' % topic)
        self.pending = remaining

    def _check_publishers(self):
        """Warn when a plotted topic gets more than one publisher.

        Duplicate publishers (a leftover teleop/bridge from another launch, a
        second simulator on the same ROS domain) interleave their values, so
        the plots show spikes towards zero.
        """
        for topic in list(self.topic_subscriptions):
            publishers = self.get_publishers_info_by_topic(topic)
            count = len(publishers)
            previous = self.publisher_counts.get(topic)
            self.publisher_counts[topic] = count
            if count > 1 and previous != count:
                names = ', '.join(sorted(set(
                    '%s/%s' % (p.node_namespace.rstrip('/'), p.node_name)
                    for p in publishers)))
                self.get_logger().warning(
                    'topic %s has %d publishers (%s): their values are '
                    'interleaved, expect spikes to zero on the plots'
                    % (topic, count, names))
            elif count == 1 and previous is not None and previous > 1:
                self.get_logger().info(
                    'topic %s has a single publisher again' % topic)

    def _on_message(self, topic, message):
        now = time.monotonic() - self.start_time
        with self.lock:
            for curve in self.attached.get(topic, []):
                value = _field_value(message, curve['field'])
                if value is None:
                    continue
                curve['times'].append(now)
                curve['values'].append(value)
                limit = now - self.window_seconds
                while curve['times'] and curve['times'][0] < limit:
                    curve['times'].popleft()
                    curve['values'].popleft()

    # -- plotting ----------------------------------------------------------

    def _build_figure(self, title):
        count = max(1, len(self.curves))
        # Many curves (e.g. the odometry perspective with the IMU channels) do
        # not fit into a single column, so switch to two columns above a
        # threshold. The window scrolls, so the figure keeps its natural
        # (possibly taller than the screen) size.
        columns = 2 if count > 6 else 1
        rows = (count + columns - 1) // columns
        height = max(2.5, 1.4 * rows)
        self.fig = Figure(figsize=(11.0 if columns == 2 else 9.0, height))
        self.fig.suptitle(title)

        first = None
        self.visible_axes = []
        for index in range(count):
            axis = self.fig.add_subplot(rows, columns, index + 1,
                                        sharex=first)
            if first is None:
                first = axis
            self.visible_axes.append(axis)
        self.fig.subplots_adjust(left=0.18 if columns == 2 else 0.25,
                                 right=0.97, top=0.94, bottom=0.7 / height,
                                 hspace=0.45, wspace=0.25)

        for axis, curve in zip(self.visible_axes, self.curves):
            line, = axis.plot([], [], color='#1f77b4')
            curve['line'] = line
            axis.set_ylabel(curve['spec'], fontsize=7)
            axis.grid(True, alpha=0.3)
        self.visible_axes[-1].set_xlabel('t, s')

    def start_animation(self):
        self.animation = FuncAnimation(
            self.fig, self._update, interval=1000.0 / self.update_rate,
            blit=False, cache_frame_data=False)

    def show_window(self):
        """Show the figure in a scrollable Tk window (mouse wheel / Page keys)."""
        try:
            root = tk.Tk()
        except tk.TclError as error:
            self.get_logger().warning(
                'cannot open a window (%s): only data collection runs' % error)
            while rclpy.ok():
                time.sleep(0.2)
            return
        root.title(str(self.get_parameter('window_title').value))

        scrollbar = tk.Scrollbar(root, orient='vertical')
        canvas = tk.Canvas(root, yscrollcommand=scrollbar.set,
                           highlightthickness=0)
        scrollbar.config(command=canvas.yview)
        scrollbar.pack(side='right', fill='y')
        canvas.pack(side='left', fill='both', expand=True)

        width, height = self.fig.get_size_inches() * self.fig.dpi
        root.geometry('%dx%d' % (min(int(width) + 20, 1900),
                                 min(int(height) + 20, 1000)))
        canvas.config(scrollregion=(0, 0, width, height))

        figure_canvas = FigureCanvasTkAgg(self.fig, master=canvas)
        figure_canvas.draw()
        canvas.create_window(0, 0, anchor='nw',
                             window=figure_canvas.get_tk_widget())

        def scroll(units):
            canvas.yview_scroll(units, 'units')

        root.bind_all('<MouseWheel>',
                      lambda event: scroll(-1 if event.delta > 0 else 1))
        root.bind_all('<Button-4>', lambda _event: scroll(-1))
        root.bind_all('<Button-5>', lambda _event: scroll(1))
        root.bind_all('<Prior>', lambda _event: scroll(-1))  # Page Up
        root.bind_all('<Next>', lambda _event: scroll(1))    # Page Down
        root.bind_all('<Home>', lambda _event: canvas.yview_moveto(0.0))
        root.bind_all('<End>', lambda _event: canvas.yview_moveto(1.0))
        root.bind_all('<q>', lambda _event: root.destroy())

        self.start_animation()
        root.mainloop()

    def _update(self, _frame):
        now = time.monotonic() - self.start_time
        with self.lock:
            curves = [
                (curve['line'], list(curve['times']), list(curve['values']))
                for curve in self.curves
            ]
        for line, times, values in curves:
            line.set_data(times, values)

        for axis in self.visible_axes:
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)
        left = max(0.0, now - self.window_seconds)
        self.visible_axes[-1].set_xlim(left, max(now, left + 1.0))
        return []


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DebugPlot()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        if matplotlib.get_backend().lower().startswith('tk'):
            node.show_window()
        else:
            node.get_logger().warning(
                'TkAgg backend is not available: no windows are shown, only '
                'data collection runs (use a display/X11 to see the plots)')
            while rclpy.ok():
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
