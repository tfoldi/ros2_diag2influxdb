# Copyright 2023, Tamas Foldi
#
# Redistribution and use in source and binary forms, with or without modification, are permitted
# provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of
#    conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of
#    conditions and the following disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to
#    endorse or promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from collections import deque, defaultdict
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos_overriding_options import QoSOverridingOptions

# from rcl_interfaces.msg import ParameterDescriptor

from diagnostic_msgs.msg import DiagnosticArray

from influxdb_client import InfluxDBClient, WriteOptions


class Diag2InfluxdbNode(Node):
    def __init__(self):
        super().__init__("diag2influxdb_node")

        self.declare_parameter("influx_bucket", "default")
        self.declare_parameter("influx_token", "")
        self.declare_parameter("influx_url", "http://influxdb:8086")
        self.declare_parameter("influx_org_id", "")
        self.declare_parameter("influx_measurement", "diagnostics")
        self.declare_parameter("influx_message_queue", 5_000)
        self.declare_parameter("influx_push_timer_period", 5.0)
        self.declare_parameter("use_current_time", False)
        self.declare_parameter("coalesce", False)

        self.subscription = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.listener_callback,
            rclpy.qos.qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )
        self.subscription  # prevent unused variable warning

        self.influx_bucket = (
            self.get_parameter("influx_bucket").get_parameter_value().string_value
        )
        token = self.get_parameter("influx_token").get_parameter_value().string_value
        influx_url = self.get_parameter("influx_url").get_parameter_value().string_value
        self.org_id = (
            self.get_parameter("influx_org_id").get_parameter_value().string_value
        )
        self.measurement = (
            self.get_parameter("influx_measurement").get_parameter_value().string_value
        )
        self.influx_message_queue = (
            self.get_parameter("influx_message_queue")
            .get_parameter_value()
            .integer_value
        )
        influx_push_timer_period = (
            self.get_parameter("influx_push_timer_period")
            .get_parameter_value()
            .double_value
        )
        self.use_current_time = (
            self.get_parameter("use_current_time").get_parameter_value().bool_value
        )
        self.coalesce = self.get_parameter("coalesce").get_parameter_value().bool_value

        self.influx_client = InfluxDBClient(
            url=influx_url, token=token, org=self.org_id, debug=False
        )

        self.get_logger().info(
            "Influxdb initalized with time period: %s" % influx_push_timer_period
        )
        self.timer = self.create_timer(influx_push_timer_period, self.timer_callback)

        self.messages = deque()

    def return_val(self, s):
        try:
            return int(s)
        except ValueError:
            pass

        try:
            return float(s)
        except ValueError:
            pass

        return s

    def timer_callback(self):
        self.get_logger().debug(
            "[Timer] flushing messages. Number of messages: %d" % len(self.messages)
        )
        if len(self.messages) > 0:
            self.write_to_influx()

    def listener_callback(self, val):
        for status in val.status:
            message = {
                "measurement": self.measurement,
                "tags": {"name": status.name, "id": status.hardware_id},
                "fields": {
                    item.key: self.return_val(item.value) for item in status.values
                },
                "time": self.get_clock().now().nanoseconds
                if self.use_current_time
                else status.header.stamp.nanoseconds,
            }
            self.messages.append(message)

        if len(self.messages) > self.influx_message_queue:
            self.write_to_influx()

    def write_to_influx(self):
        self.get_logger().debug("Writing %d messages to InfluxDB" % len(self.messages))

        message_buffer = []
        for _ in range(len(self.messages)):
            message_buffer.append(self.messages.popleft())

        if self.coalesce:
            message_buffer = self.aggregate_messages(message_buffer, method="mean")

        with self.influx_client.write_api(
            write_options=WriteOptions(
                batch_size=2000,
                flush_interval=1000,
                jitter_interval=0,
                retry_interval=5_000,
                max_retries=0,
            )
        ) as _write_client:
            _write_client.write(self.influx_bucket, self.org_id, record=message_buffer)

    def aggregate_messages(self, messages, method="last"):
        # Check if messages list is empty
        if not messages:
            return []

        last_time = messages[-1]["time"]

        # Initialize a dictionary to hold the groups
        groups = defaultdict(
            lambda: {
                "measurement": "",
                "tags": {},
                "fields": defaultdict(list),
                "time": 0,
            }
        )

        # Group the messages by tag combinations
        for message in messages:
            tags_tuple = tuple(
                sorted(message["tags"].items())
            )  # Convert tags dict to a sortable tuple
            group = groups[tags_tuple]
            group["measurement"] = message["measurement"]
            group["tags"] = message["tags"]
            group["time"] = last_time
            for field, value in message["fields"].items():
                group["fields"][field].append(value)

        # Aggregate the field values within each group
        aggregated_data = []
        for tags_tuple, group in groups.items():
            for field, values in group["fields"].items():
                if method == "last":
                    group["fields"][field] = values[-1]  # Use the last value
                elif method == "mean":
                    group["fields"][field] = np.mean(values)  # Compute the mean
            group["fields"] = dict(group["fields"])
            aggregated_data.append(group)

        self.get_logger().debug("Aggregated data: %s" % aggregated_data)

        return [aggregated_data]


def main(args=None):
    rclpy.init(args=args)
    node = Diag2InfluxdbNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
