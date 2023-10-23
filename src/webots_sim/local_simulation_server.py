#!/usr/bin/env python3

# Copyright 1996-2022 Cyberbotics Ltd.
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

"""Local simulation server."""

import os
import socket
import subprocess
import sys

HOST = ''  # Any host can connect
PORT = 2000 if len(sys.argv) < 2 else int(sys.argv[1])  # Port to listen on


def close_connection(connection, message):
    connection.sendall(message.encode('utf-8'))
    print(message, file=sys.stderr)
    connection.close()


tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

tcp_socket.bind((HOST, PORT))
tcp_socket.listen()
while True:
    print(f'Waiting for connection on port {PORT}...')
    connection, address = tcp_socket.accept()

    print(f'Connection from {address}')
    data = connection.recv(1024)
    command = data.decode('utf-8').split(' ')

    if not command[0].endswith('webots'):
        message = f'FAIL: \'{command[0]}\' is not recognized as a Webots executable.'
        close_connection(connection, message)
        continue
    else:
        if not os.path.isabs(command[0]) and command[0] != 'webots':
            message = f'FAIL: \'{command[0]}\' must be either \'webots\' or an absolute path to the executable.'
            close_connection(connection, message)
            continue

    if os.path.isabs(command[0]):
        pass
    elif 'WEBOTS_HOME' in os.environ:
        path_suffix = 'Contents/MacOS/webots' if sys.platform == 'darwin' else 'webots'
        command[0] = os.path.join(os.environ['WEBOTS_HOME'], path_suffix)
    else:
        message = 'FAIL: WEBOTS_HOME environment variable is not defined. Please define a valid Webots installation folder.'
        close_connection(connection, message)
        continue

    invalid_world_file = False
    for argument in command:
        if argument.endswith('.wbt'):
            if not os.path.isfile(argument):
                message = f'FAIL: The world file \'{argument}\' doesn\'t exist.'
                close_connection(connection, message)
                invalid_world_file = True
            break
    if invalid_world_file:
        continue

    try:
        webots_process = subprocess.Popen(command)
    except FileNotFoundError:
        message = f'FAIL: \'{command[0]}\' could not be found on the host.'
        close_connection(connection, message)
        continue

    connection.sendall(b'ACK')
    connection.settimeout(1)
    connection_closed = False
    while webots_process.poll() is None:
        try:
            data = connection.recv(1024)
        except socket.timeout:
            continue
        else:
            if not data:
                print('Connection was closed by the client.')
                connection.close()
                webots_process.kill()
                connection_closed = True
                break

    if connection_closed:
        connection_closed = False
        continue

    print('Webots was executed successfully.')
    closing_message = 'CLOSED'
    connection.sendall(closing_message.encode('utf-8'))
    connection.close()
