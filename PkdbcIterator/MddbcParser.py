# -*- coding: utf-8 -*- 
'''
Created on 9th Mar, 2018

@author: Alvin Ye

'''
from PkdbcCoreModel import MdCoreModel
import re


class DbcParser(object):
    '''
    this class acts as interface between dbc file and dbcCoreModels through regular express engine.
    '''

    def __init__(self):
        '''
        Constructor
        '''
        self._can_network = MdCoreModel.CanNetwork()
        self._keyWords = {'version': self.version_hdl,
                          'BU_': self.node_hdl,
                          'BO_ ': self.msg_hdl,
                          ' SG_ ': self.signal_hdl}
        self._current_node = None
        self._current_msg = None
        self._current_signal = None
        self._patten = ''
        self._current_msg_sender = None
        self._can_networkversion = None

    def parser_dbc(self, file):
        with open(file, "rb") as f:
            for line in f:
                line = str(line).strip().strip("b'")
                # below is to remove \r\n
                line = line[:-4]
                self.parse_line(line)

    def parse_line(self, line):
        for key, func in self._keyWords.items():
            if line.startswith(key):
                line = line.strip()
                func(line)

    def version_hdl(self, line):
        self._patten = 'VERSION\s+"(?P<version>\S+)"'
        reg = re.search(self._patten, line)
        try:
            self._can_networkversion = reg.group('version')
        except AttributeError:
            print("[WARINING]: version definition violation")

    def node_hdl(self, line):
        self._patten = 'BU_\s*:\s*(?P<nodes>.+)\s*'
        reg = re.search(self._patten, line)
        reg = re.sub('\s+', ' ', reg.group('nodes')).strip()
        try:
            for node_name in reg.split(' '):
                self._can_network.append_nodes(node_name, MdCoreModel.CanNode(node_name))
        except AttributeError:
            print('[WARINING]: node definition violation')

    def msg_hdl(self, line):
        self._patten = 'BO_\s+(?P<msg_id>\d+)\s+(?P<msg_name>\S+)\s*:\s*(?P<length>\d+)\s+(?P<msg_sender>\S+)'
        reg = re.search(self._patten, line)
        try:
            self._current_msg_sender = self._can_network.nodes[reg.group('msg_sender')]
        except KeyError:
            print(
                '[WARINING]: the msg sender {0} is not in the list of node definition'.format(reg.group('msg_sender')))
            self._current_msg_sender = MdCoreModel.CanNode(reg.group('msg_sender'))
            self._can_network.append_nodes(self._current_msg_sender.node_name, self._current_msg_sender)
        # create a message object
        try:
            self._current_msg = MdCoreModel.CanMsg(int(reg.group('msg_id')), reg.group('msg_name'),
                                                   int(reg.group('length')), msg_sender=self._current_msg_sender)
        except AttributeError:
            print("[WARINING] message definition violation")
        # add this message to parent node's tx message
        self._can_network.nodes[self._current_msg.msg_sender.node_name].append_tx_msg(self._current_msg.msg_name,
                                                                                      self._current_msg)

    def signal_hdl(self, line):
        self._patten = 'SG_\s+(?P<name>\S+)\s*(?P<multiplexer>M)?(?P<multiplexer_id>m\d+)?\s*:\s*'
        self._patten += '(?P<start_bit>\d+)\|(?P<length>\d+)\@(?P<little_endian>[0|1])(?P<signed>[\+|\-])\s*'
        self._patten += '\(\s*(?P<factor>\S+)\s*,\s*(?P<offset>\S+)\s*\)\s*'
        self._patten += '\[\s*(?P<value_min>\S+)\s*\|\s*(?P<value_max>\S+)\s*\]\s*"?(?P<unit>[^"]*)?"?\s+(?P<receivers>.+)'
        reg = re.search(self._patten, line)
        # create a signal object
        try:
            self._current_signal = MdCoreModel.CanSignal(reg.group('name'), int(reg.group('start_bit')),
                                                         int(reg.group('length')), int(reg.group('little_endian')),
                                                         reg.group('signed'),
                                                         float(reg.group('factor')), float(reg.group('offset')),
                                                         float(reg.group('value_min')), float(reg.group('value_max')),
                                                         reg.group('unit'),
                                                         reg.group('multiplexer'), reg.group('multiplexer_id'),
                                                         msg_carrier=self._current_msg)
        except AttributeError:
            print("[WARNING]: signal definition violation")
        # add this signal to parent msg
        self._current_msg.append_signal(self._current_signal.signal_name, self._current_signal)
        # find signal receivers, and then add current msg to receivers' rx msgs
        for receiver in reg.group('receivers').split(','):
            if receiver in self._can_network.nodes.keys():
                self._can_network.nodes[receiver].append_rx_msg(self._current_msg.msg_name, self._current_msg)


    def iterate_dbc(self, f):
        for node_name, node in self._can_network.nodes.items():
            f.write("{0}\n".format(node_name))
            for msg_name, msg in node.node_tx_msgs.items():
                f.write("\t{0}\n".format(msg_name))
                for signal_name, signal in msg.msg_signals.items():
                    f.write(
                        "\t\t{0}, start:{1}, len:{2}, endian:{3}, sign:{4}, factor:{5}, offset:{6}, Min:{7}, Max:{8}, unit:{9}\n"
                            .format(signal_name, signal.signal_start_bit, signal.signal_len,
                                    signal.signal_little_endian,
                                    signal.signal_signed, signal.signal_factor, signal.signal_offset,
                                    signal.signal_min_values, signal.signal_max_value, signal.signal_unit))

    @property
    def can_network(self):
        return self._can_network
