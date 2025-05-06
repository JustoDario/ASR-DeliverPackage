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
from navigation_system_interfaces.srv import SetMap


class SetMapVerb(VerbExtension):
    """Set map."""
    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            'map',
            help='Path to the map file')

    def main(self, *, args):
        with NodeStrategy(args) as node:
            client = node.create_client(
                SetMap, '/navigation_system_node/set_map')
            client.wait_for_service()
            request = SetMap.Request()
            request.map_path = args.map
            client.call_async(request)
        return 0

