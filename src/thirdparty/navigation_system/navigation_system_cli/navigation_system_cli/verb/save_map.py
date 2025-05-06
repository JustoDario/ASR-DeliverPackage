# Copyright 2024 Juan Carlos Manzanares Serrano
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

from ros2cli.verb import VerbExtension
from ros2cli.node.strategy import NodeStrategy
from ros2cli.node.strategy import add_arguments
from slam_toolbox.srv import SaveMap


class SaveMapVerb(VerbExtension):
    """Save map."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            'name',
            nargs='?',
            default=None,
            help='Name of the map file (optional)',)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            node.get_logger().info('Save map')
            client = node.create_client(
                SaveMap, 'slam_toolbox/save_map')
            client.wait_for_service()
            request = SaveMap.Request()
            if args.name is not None:
                request.name.data = args.name

            client.call_async(request)
        return 0
