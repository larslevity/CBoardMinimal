#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 20 15:18:21 2018
@author: bianca
"""

import socket
import abc

try:
    from Src.Communication import pickler
except ImportError:
    print('Relative Import does not work..')


class Socket(object):  # pragma: no cover
    """Base class for Sockets. This defines the interface to Sockets"""
    __metaclass__ = abc.ABCMeta

    def __init__(self, ip):
        self.client_socket = socket.socket()
        self.client_socket.connect((ip, 12397))
        self.connection = self.client_socket.makefile('wb')

    def send_all(self, data):
        self.client_socket.sendall(pickler.pickle_data(data))

    def recieve_data(self):
        ans = pickler.unpickle_data(self.client_socket.recv(4096))
        return ans

    def close(self):
        self.send_all(['Exit'])
        self.connection.close()
        self.client_socket.close()


class LivePlotterSocket(Socket):
    def __init__(self, ip='134.28.136.70'):
        Socket.__init__(self, ip)

    def send_sample(self, sample):
        self.send_all(['sample', sample])
        resp = self.recieve_data()
        return resp
