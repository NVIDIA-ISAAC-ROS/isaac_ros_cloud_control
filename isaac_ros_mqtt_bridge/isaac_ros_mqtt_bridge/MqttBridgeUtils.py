# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

"""This module provides utility functions for case conversion between MQTT and ROS."""

import datetime
from enum import Enum
import json
import re


class State(Enum):
    ONLINE = 0
    OFFLINE = 1
    CONNECTIONBROKEN = 2


class ConnectionMessage:
    """Connection message defined in VDA5050 spec."""

    counter = 0

    def __init__(self, manufacturer: str, serialNumber: str, connectionState: State):
        self.headerId = ConnectionMessage.counter
        ConnectionMessage.counter += 1
        self.timestamp = datetime.datetime.now().isoformat()
        self.version = '2.1.0'
        self.manufacturer = manufacturer
        self.serialNumber = serialNumber
        self.connectionState = connectionState.name

    def __str__(self):
        return json.dumps(self.__dict__)


def convert_camel_to_snake(camel_case_string: str) -> str:
    """
    Convert a string from camel case to snake case.

    Parameters
    ----------
    camel_case_string : str
        The given string to convert.

    Returns
    -------
    str
        The new string in snake_case.
    """
    return re.sub(r'(?<!^)(?=[A-Z])', '_', camel_case_string).lower()


def convert_snake_to_camel(snake_case_string: str, dromedary: bool = False) -> str:
    """
    Convert a string from snake case to camel case.

    Parameters
    ----------
    snake_case_string : str
        The given string to convert.
    dromedary : bool, optional
        Set whether the conversion should be dromedary case (initial letter as lowercase),
        by default False.

    Returns
    -------
    str
        The new string in camel (or dromedary) case.
    """
    if len(snake_case_string) == 0:
        return snake_case_string
    word_list = []
    for count, word in enumerate(snake_case_string.split('_')):
        if dromedary and count == 0:
            word_list.append(word[0].lower() + word[1:])
        else:
            word_list.append(word[0].upper() + word[1:])
    return ''.join(word_list)


def convert_dict_keys(dictionary: dict, case_converter: str) -> dict:
    """
    Convert the keys of the dictionary based on the given case converter recursively.

    Parameters
    ----------
    dictionary : dict
        The given dictionary to convert the keys for.
    case_converter : str
        The defined case conversion for this function. Can only be 'snake_to_dromedary',
        'snake_to_camel', and 'camel_to_snake'.

    Returns
    -------
    dict
        The new dictionary with the converted keys.

    Raises
    ------
    AssertionError
        case_converter string must be one of the three strings defined.
    """
    new_dict = {}
    if case_converter == 'snake_to_dromedary':
        def convert_function(key): return convert_snake_to_camel(key, True)
    elif case_converter == 'snake_to_camel':
        def convert_function(key): return convert_snake_to_camel(key, False)
    elif case_converter == 'camel_to_snake':
        def convert_function(key): return convert_camel_to_snake(key)
    else:
        raise AssertionError('case_converter is invalid.')

    for k, v in dictionary.items():
        new_key = convert_function(k)
        if isinstance(v, dict):
            v = convert_dict_keys(v, case_converter)
        if isinstance(v, list):
            new_value = []
            for item in v:
                if isinstance(item, dict):
                    new_value.append(
                        convert_dict_keys(item, case_converter))
                else:
                    new_value.append(v)
            v = new_value
        assert new_key not in new_dict
        new_dict[new_key] = v
    return new_dict
