# -*- coding: utf-8 -*- 
'''
Created on 9th Mar, 2018

@author: Alvin Ye

this is a light-weight dbc abstraction of minimal core CAN elements
'''


class CanNetwork(object):
    '''
    classdocs
    '''

    def __init__(self, baud_rate=None, version=None):
        '''
        Constructor
        '''
        self._baud_rate = baud_rate
        self._version = version
        self._nodes = {}

    @property
    def version(self):
        return self._version

    @version.setter
    def version(self, version):
        self._version = version

    @property
    def nodes(self):
        return self._nodes

    def append_nodes(self, node_name, node):
        self._nodes[node_name] = node


class CanNode(object):
    '''
    classsdocs
    '''

    def __init__(self, node_name, node_id=None, tx_msgs=None, rx_msgs=None):
        self._node_id = node_id
        self._node_name = node_name
        self._txMsgs = {}
        self._rxMsgs = {}

    @property
    def node_id(self):
        return self._node_id

    @property
    def node_name(self):
        return self._node_name

    @property
    def node_tx_msgs(self):
        return self._txMsgs

    def append_tx_msg(self, tx_msg_name, tx_msg):
        self._txMsgs[tx_msg_name] = tx_msg

    @property
    def node_rx_msgs(self):
        return self._rxMsgs

    def append_rx_msg(self, rx_msg_name, rx_msg):
        self._rxMsgs[rx_msg_name] = rx_msg


class CanMsg(object):
    '''
    classdocs
    '''

    def __init__(self, msg_id, msg_name, msg_len, msg_type=None, msg_sender=None, msg_receivers=None,
                 msg_send_type=None):
        '''
        Constructor
        '''
        self._msg_id = msg_id
        self._msg_name = msg_name
        self._msg_len = msg_len
        self._msg_type = msg_type
        self._msg_sender = msg_sender
        self._msg_receivers = msg_receivers
        self._msg_send_type = msg_send_type
        self._signals = {}

    @property
    def msg_id(self):
        return self._msg_id

    @property
    def msg_name(self):
        return self._msg_name

    @msg_name.setter
    def msg_name(self, msg_name):
        self._msg_name = msg_name

    @property
    def msg_len(self):
        return self._msg_len

    @property
    def msg_type(self):
        return self._msg_type

    @property
    def msg_sender(self):
        return self._msg_sender

    @msg_sender.setter
    def msg_sender(self, node):
        self._msg_sender = node

    @property
    def msg_send_type(self):
        return self._msg_send_type

    @property
    def msg_receivers(self):
        return self._msg_receivers

    def append_msg_receiver(self, node_name, node):
        self._msg_receivers[node_name] = node

    @property
    def msg_signals(self):
        return self._signals

    def append_signal(self, signal_name, signal):
        self._signals[signal_name] = signal
        signal.signal_msg_carrier = self


class CanSignal(object):
    '''
    classdocs
    '''

    def __init__(self, name, start_bit, length, little_endian=1, signed=False, factor=1.0, offset=0.0, value_min=0.0,
                 value_max=0.0, unit="", multiplexer=None, multiplexer_id=None, msg_carrier=None):
        self._name = name
        self._start_bit = start_bit
        self._length = length
        self._little_endian = little_endian
        self._signed = signed
        self._factor = factor
        self._offset = offset
        self._value_min = value_min
        self._value_max = value_max
        self._unit = unit
        self._multiplexer = multiplexer
        self._multiplexer_id = multiplexer_id
        self._msg_carrier = msg_carrier

    @property
    def signal_name(self):
        return self._name

    @property
    def signal_start_bit(self):
        return self._start_bit

    @signal_start_bit.setter
    def signal_start_bit(self, start_bit):
        self._start_bit = start_bit

    @property
    def signal_len(self):
        return self._length

    @signal_len.setter
    def signal_len(self, length):
        self._length = length

    @property
    def signal_little_endian(self):
        return self._little_endian

    @property
    def signal_signed(self):
        return self._signed

    @property
    def signal_factor(self):
        return self._factor

    @property
    def signal_offset(self):
        return self._offset

    @property
    def signal_min_values(self):
        return self._value_min

    @property
    def signal_max_value(self):
        return self._value_max

    @property
    def signal_unit(self):
        return self._unit

    @property
    def signal_multiplexer(self):
        return self._multiplexer

    @signal_multiplexer.setter
    def signal_multiplexer(self, multiplexer):
        self._multiplexer = multiplexer

    @property
    def signal_multiplexer_id(self):
        return self._multiplexer_id

    @signal_multiplexer_id.setter
    def signal_multiplexer_id(self, multiplexer_id):
        self._multiplexer_id = multiplexer_id

    @property
    def signal_msg_carrier(self):
        return self._msg_carrier

    @signal_msg_carrier.setter
    def signal_msg_carrier(self, msg_carrier):
        self._msg_carrier = msg_carrier
