#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import numpy
import scipy.spatial
from matplotlib import pyplot

from hdl_localization.msg import ScanMatchingStatus


class Plotter(Node):
    def __init__(self):
        super().__init__("status_plotter")

        pyplot.ion()
        pyplot.show(block=False)

        self.status_buffer = []
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.status_sub = self.create_subscription(
            ScanMatchingStatus, "/status", self.status_callback, QoSProfile(history=2)
        )

    def status_callback(self, status_msg):
        self.status_buffer.append(status_msg)

        if len(self.status_buffer) > 50:
            self.status_buffer = self.status_buffer[-50:]

    def timer_callback(self):
        if len(self.status_buffer) < 2:
            return

        errors = {}
        for status in self.status_buffer:
            for label, error in zip(status.prediction_labels, status.prediction_errors):
                if label.data not in errors:
                    errors[label.data] = []

                quat = [
                    error.rotation.x,
                    error.rotation.y,
                    error.rotation.z,
                    error.rotation.w,
                ]
                trans = [error.translation.x, error.translation.y, error.translation.z]

                t = status.header.stamp.sec + status.header.stamp.nanosec / 1e9
                t_error = numpy.linalg.norm(trans)
                r_error = numpy.linalg.norm(
                    scipy.spatial.transform.Rotation.from_quat(quat).as_rotvec()
                )

                if len(errors[label.data]) and abs(errors[label.data][-1][0] - t) > 1.0:
                    errors[label.data] = []

                errors[label.data].append((t, t_error, r_error))

        pyplot.clf()
        for label in errors:
            errs = numpy.float64(errors[label])
            pyplot.subplot(211)
            pyplot.plot(errs[:, 0], errs[:, 1], label=label)

            pyplot.subplot(212)
            pyplot.plot(errs[:, 0], errs[:, 2], label=label)

        pyplot.subplot(211)
        pyplot.ylabel("trans error")
        pyplot.subplot(212)
        pyplot.ylabel("rot error")

        pyplot.legend(loc="upper center", bbox_to_anchor=(0.5, -0.05), ncol=len(errors))
        pyplot.gcf().canvas.flush_events()
        # pyplot.pause(0.0001)


def main():
    rclpy.init()

    node = Plotter()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
