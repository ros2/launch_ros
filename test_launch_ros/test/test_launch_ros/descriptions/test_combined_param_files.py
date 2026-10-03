# Copyright 2026 Metro Robots
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

"""Tests for launch_ros.descriptions.CombinedParameterFiles."""

from launch_ros.descriptions import CombinedParameterFiles
from test_parameter_file import get_parameter_file, MockContext
import yaml


def get_params(lc, desc):
    evaluated_param_file = desc.evaluate(lc)
    evaluated_values = yaml.safe_load(open(evaluated_param_file))
    return evaluated_values.get('/my_ns/my_node', {}).get('ros__parameters')


def test_combined_description():
    param_file_dict1 = {
        '/my_ns/my_node': {
            'ros__parameters': {
                'indie_param1': 1,
                'repeated_param': 1,
            }
        }
    }
    param_file_dict2 = {
        '/my_ns/my_node': {
            'ros__parameters': {
                'indie_param2': 2,
                'repeated_param': 2,
            }
        }
    }

    lc = MockContext()
    with get_parameter_file(yaml.dump(param_file_dict1)) as file_name1, \
         get_parameter_file(yaml.dump(param_file_dict2)) as file_name2:

        # Load parameters with dict1 first
        desc_a = CombinedParameterFiles([file_name1, file_name2])
        params_a = get_params(lc, desc_a)
        assert isinstance(params_a, dict)
        assert params_a.get('indie_param1') == 1
        assert params_a.get('indie_param2') == 2
        assert params_a.get('indie_param3') is None
        assert params_a.get('repeated_param') == 2

        # Load parameters with dict2 first
        desc_b = CombinedParameterFiles([file_name2, file_name1])
        params_b = get_params(lc, desc_b)
        assert isinstance(params_b, dict)
        assert params_b.get('indie_param1') == 1
        assert params_b.get('indie_param2') == 2
        assert params_b.get('indie_param3') is None
        assert params_b.get('repeated_param') == 1
