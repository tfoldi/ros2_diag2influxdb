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

import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

from influxdb_client import InfluxDBClient, WriteOptions


class Diag2InfluxdbNode(Node):
    def __init__(self):
        super().__init__("diag2influxdb_node")

        qos_history_desc = ParameterDescriptor(description="QoS history depth")
        self.declare_parameter("qos_history", 100, qos_history_desc)

        qos_history = (
            self.get_parameter("qos_history").get_parameter_value().integer_value
        )

        self.subscription = self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self.listener_callback,
            qos_history,
        )
        self.subscription  # prevent unused variable warning

        self.influx_bucket = "antares"
        token = "3f1cZRrR4mMOacWR5ManKYCKfFEyQqNf"
        influx_url = "http://127.1.27.3"
        self.org_id = "e6cd7430a342269a"

        self.influx_client = InfluxDBClient(
            url=influx_url, token=token, org=self.org_id, debug=False
        )

        self.messages = []

    def listener_callback(self, msg):
        self.get_logger().debug('received message: "%s/%s"' % (msg.id, hex(msg.id)))

        message = {
            "measurement": "Dodgem",
            "fields": {
                k: str(v)
                if isinstance(v, cantools.database.namedsignalvalue.NamedSignalValue)
                else v
                for k, v in val.items()
            },
            # "time": time.time() * 1000 * 1000000,
        }

        self.messages.append(message)

        if len(self.messages) > 2000:
            with self.influx_client.write_api(
                write_options=WriteOptions(
                    batch_size=2000,
                    flush_interval=1000,
                    jitter_interval=0,
                    retry_interval=5_000,
                    max_retries=0,
                )
            ) as _write_client:
                _write_client.write(
                    self.influx_bucket, self.org_id, record=self.messages
                )
                self.messages = []

        # self.messages.append(message)

        # if len(self.messages) > 2000:
        #     with self.influx_client.write_api(
        #         write_options=WriteOptions(
        #             batch_size=2000,
        #             flush_interval=1000,
        #             jitter_interval=0,
        #             retry_interval=5_000,
        #             max_retries=0,
        #         )
        #     ) as _write_client:
        #         _write_client.write(
        #             self.influx_bucket, self.org_id, record=self.messages
        #         )
        #         self.messages = []


def main(args=None):
    rclpy.init(args=args)
    node = Diag2InfluxdbNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
