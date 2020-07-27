'''
Created on 12th Mar, 2018

@author: Alvin Ye
'''
from PkdbcCoreModel.MdCoreModel import CanSignal
from PkdbcCoreModel.MdCoreModel import CanMsg
from PkdbcCoreModel.MdCoreModel import CanNode
import copy


class CanMsgLayoutComposer(object):
    '''
    supporting all kinds of gaps, different endian-ness, multiple-byte, multiplexer
    '''

    def __init__(self, msg):
        '''
        Constructor
        '''
        self._msg = msg
        self._sorted_signals = {}
        # just need a empty msg
        self.composed_msg = CanMsg(msg.msg_id, msg.msg_name, msg.msg_len, msg.msg_type, msg.msg_sender,
                                   msg.msg_receivers, msg.msg_send_type)

    def _sort_signal(self):
        '''
        to sort the signals in the msg by start bit and store them in self._sorted_signals
        '''
        for signal in self._msg.msg_signals.values():
            self._sorted_signals[signal.signal_start_bit] = signal
            #  sort signals in the dict by start bit, but return a list of tuples!
        self._sorted_signals = sorted(self._sorted_signals.items(), key=lambda d: d[0])

    def compose_msg_with_multiplexer(self):
        '''this func aims to compose multi-msg for multiplexer
        each msg is called virtual_msg
        a container called virtual_node is created to hold these virtual msgs
        :return a virtual node that carries virtual msgs by different multiplexer id
        '''

        multiplexer_ids = []
        virtual_node = CanNode('_virtual_node')

        # first iteration to create all virtual msgs first
        for signal in self._msg.msg_signals.values():
            # if it has a multiplexer
            if signal.signal_multiplexer_id:
                if signal.signal_multiplexer_id not in multiplexer_ids:
                    virtual_msg = CanMsg(self._msg.msg_id, self._msg.msg_name + signal.signal_multiplexer_id,
                                         self._msg.msg_len)
                    virtual_msg.append_signal(signal.signal_name,signal)
                    virtual_node.append_tx_msg(virtual_msg.msg_name, virtual_msg)
                    multiplexer_ids.append(signal.signal_multiplexer_id)
                else:
                    virtual_node.node_tx_msgs[self._msg.msg_name + signal.signal_multiplexer_id].append_signal(
                        signal.signal_name, signal)
        # second iteration to put all signal without multiplexer id to virtual msgs
        # in case such signal is iterated before virtual msgs created
        for signal in self._msg.msg_signals.values():
            # if it has a multiplexer
            if not signal.signal_multiplexer_id:
                for msg in virtual_node.node_tx_msgs.values():
                    msg.append_signal(signal.signal_name, signal)
        return virtual_node

    def compose_msg_without_multiplexer(self):
        '''this func aims to compose a new msg based on original msg with gap signal and sub signals that from a
        msg that is over byte boundary or multi-byte signal
        :return this new msg as described
        '''
        # once used,that bit will be filled by '1'
        bits_in_byte = 8
        used_bits = [[0 for i in range(bits_in_byte)] for j in range(self._msg.msg_len)]
        gap_syntax = 'unused'
        gap_idx = 0
        sub_signal_idx = 0
        sub_signal = None
        gap = None
        # sub signals always has the original signal at the beginning of the list
        # so that code generation for across-byte signal access macro will be easier
        sub_signals = []
        # this group just wrap sub signals into a list for easy iteration, so it is a list of list
        sub_signals_group = []

        # sort signals in the msg by start bit, super important!!!
        self._sort_signal()
        # iterate each byte in the msg
        for byteIdx in range(0, self._msg.msg_len):
            # create a copy in order not to change the indexer
            byte_idx_temp = byteIdx
            # now iterate each signal in the msg 
            for (start_bit, signal) in self._sorted_signals:
                #  now check if the signal's start bit falls in the byte from outer loop
                if byteIdx * 8 <= start_bit <= (byteIdx + 1) * 8 - 1:
                    #  is it Intel?
                    if signal.signal_little_endian > 0:
                        #  is it multi-byte or across byte boundary?
                        # CAUTION: here use byte_idx, in while hoop for multi-byte,it is byte_idx_temp used
                        if sum(used_bits[byteIdx]) + signal.signal_len < 9:
                            #  check any gap to the left of the signal ?
                            bit_offset = start_bit % 8
                            gap_len = used_bits[byteIdx][:bit_offset].count(0)
                            if gap_len > 0:
                                #  create gap signal
                                gap = CanSignal(gap_syntax + str(gap_idx),
                                                start_bit - gap_len,
                                                gap_len,
                                                little_endian=1,
                                                msg_carrier=signal.signal_msg_carrier)
                                # put the gap signal into composed msg
                                self.composed_msg.append_signal(gap.signal_name, gap)
                                #  set unused bits
                                for i in range(0, gap_len + 1):
                                    used_bits[byteIdx][bit_offset - i] = 1
                                gap_idx += 1
                            # just put the signal into composed msg
                            self.composed_msg.append_signal(signal.signal_name, signal)
                            #  set used bits
                            for i in range(0, signal.signal_len):
                                used_bits[byteIdx][bit_offset + i] = 1
                        else:
                            # put the original at the beginning of the list
                            sub_signals.append(copy.deepcopy(signal))
                            while signal.signal_len + sum(used_bits[byte_idx_temp]) > 8:
                                # also need to check if any gap to the left for Intel multi-byte case
                                # for motorola, not necessary, because of growth direction
                                bit_offset = signal.signal_start_bit % 8
                                gap_len = used_bits[byte_idx_temp][:bit_offset].count(0)
                                if gap_len > 0:
                                    gap = CanSignal(gap_syntax + str(gap_idx),
                                                    start_bit - gap_len,
                                                    gap_len,
                                                    little_endian=1,
                                                    msg_carrier=signal.signal_msg_carrier)
                                    # put the gap signal into composed msg
                                    self.composed_msg.append_signal(gap.signal_name, gap)
                                    #  set used bits
                                    for i in range(0, gap_len + 1):
                                        used_bits[byte_idx_temp][bit_offset - i] = 1
                                    gap_idx += 1
                                # create sub signal
                                sub_signal = CanSignal(signal.signal_name + str(sub_signal_idx),
                                                       signal.signal_start_bit,
                                                       bits_in_byte - bit_offset,
                                                       little_endian=1,
                                                       msg_carrier=signal.signal_msg_carrier)
                                # put sub signal in composed msg
                                self.composed_msg.append_signal(sub_signal.signal_name, sub_signal)
                                # put the sub signal into the list - sub signals
                                sub_signals.append(sub_signal)
                                sub_signal_idx += 1
                                #  set used bits
                                for i in range(0, sub_signal.signal_len):
                                    used_bits[byte_idx_temp][bit_offset + i] = 1
                                # remove the sub signal length
                                signal.signal_len -= sub_signal.signal_len
                                # shift  signal start bit
                                signal.signal_start_bit += sub_signal.signal_len
                                #  support CAN FD
                                if byte_idx_temp < self._msg.msg_len-1:
                                    byte_idx_temp += 1
                            else:
                                #  create signal within signal byte
                                sub_signal = CanSignal(signal.signal_name + str(sub_signal_idx),
                                                       signal.signal_start_bit, signal.signal_len,
                                                       little_endian=1,
                                                       msg_carrier=signal.signal_msg_carrier)
                                # put sub signal in composed msg
                                self.composed_msg.append_signal(sub_signal.signal_name, sub_signal)
                                # put the sub signal into the list - sub signals
                                sub_signals.append(sub_signal)
                                sub_signal_idx = 0
                                #  set used bits
                                #  caution: bit_offset is not updated due to the exit of "while"
                                #  must use signal.signal_start_bit % 8
                                for i in range(0, sub_signal.signal_len):
                                    used_bits[byte_idx_temp][sub_signal.signal_start_bit % 8 + i] = 1
                    else:
                        #  this is Motorola
                        #  check multiple-byte?
                        if sum(used_bits[byteIdx]) + signal.signal_len < 9:
                            #  check any gap to the left of the signal
                            bit_offset = start_bit % 8
                            gap_len = used_bits[byteIdx][:bit_offset + 1 - signal.signal_len].count(0)
                            if gap_len > 0:
                                gap = CanSignal(gap_syntax + str(gap_idx),
                                                start_bit - signal.signal_len,
                                                gap_len,
                                                little_endian=0,
                                                msg_carrier=signal.signal_msg_carrier)
                                # put the gap signal into composed msg
                                self.composed_msg.append_signal(gap.signal_name, gap)
                                #  set unused bits
                                for i in range(0, gap_len):
                                    used_bits[byteIdx][bit_offset - signal.signal_len - i] = 1
                                gap_idx += 1
                            # just put the signal into composed msg
                            self.composed_msg.append_signal(signal.signal_name, signal)
                            #  set used bits
                            for i in range(0, signal.signal_len):
                                used_bits[byteIdx][bit_offset - i] = 1
                        else:
                            # put the original signal at the beginning of the list
                            sub_signals.append(copy.deepcopy(signal))
                            while sum(used_bits[byte_idx_temp]) + signal.signal_len > 8:
                                bit_offset = signal.signal_start_bit % 8
                                # no need to check left gap due to motorola growth direction
                                # create sub signal
                                sub_signal = CanSignal(signal.signal_name + str(sub_signal_idx),
                                                       signal.signal_start_bit,
                                                       bit_offset+1,
                                                       little_endian=0,
                                                       msg_carrier=signal.signal_msg_carrier)
                                # put sub signal in composed msg
                                self.composed_msg.append_signal(sub_signal.signal_name, sub_signal)
                                # put the sub signal into the list - sub signals
                                sub_signals.append(sub_signal)
                                sub_signal_idx += 1
                                #  set used bits before signal length adjusted
                                for i in range(0, sub_signal.signal_len):
                                    used_bits[byte_idx_temp][bit_offset - i] = 1
                                # remove the sub signal length
                                signal.signal_len -= sub_signal.signal_len
                                # shift  signal start bit
                                # CAUTION: error prone, not add sub_signal.signal_len, but bits_in_byte
                                signal.signal_start_bit = bits_in_byte * (byte_idx_temp + 2) - 1
                                # support CAN FD
                                if byte_idx_temp < self._msg.msg_len-1:
                                    byte_idx_temp += 1
                            else:
                                # create signal within signal byte
                                sub_signal = CanSignal(signal.signal_name + str(sub_signal_idx),
                                                       signal.signal_start_bit, signal.signal_len,
                                                       little_endian=0,
                                                       msg_carrier=signal.signal_msg_carrier)
                                # put sub signal in composed msg
                                self.composed_msg.append_signal(sub_signal.signal_name, sub_signal)
                                # put the sub signal into the list - sub signals
                                sub_signals.append(sub_signal)
                                sub_signal_idx = 0
                                #  set used bits
                                #  caution: bit_offset is not updated due to the exit of "while"
                                #  must use signal.signal_start_bit % 8
                                for i in range(0, sub_signal.signal_len):
                                    used_bits[byte_idx_temp][sub_signal.signal_start_bit % 8 - i] = 1
                # sub_signals_group is used for multi-byte signal access macro - code generation
                if len(sub_signals):
                    sub_signals_group.append(copy.deepcopy(sub_signals))
                # whatever there is sub signal, sub signals reference must be cleared for next loop
                sub_signals.clear()
            # gap stuffing from right direction
            if sum(used_bits[byteIdx]) < bits_in_byte:
                gap_len = bits_in_byte - sum(used_bits[byteIdx])
                gap = CanSignal(gap_syntax + str(gap_idx),
                                sum(used_bits[byteIdx]) + 8 * byteIdx,
                                gap_len,
                                little_endian=1,
                                msg_carrier=signal.signal_msg_carrier)
                self.composed_msg.append_signal(gap.signal_name, gap)
                gap_idx += 1
                #  set used bits
                for i in range(1, gap_len + 1):
                    used_bits[byteIdx][bits_in_byte - i] = 1
        # composed_msg is sorted, containing original signals and sub signals if any, and gap signal
        return self.composed_msg, sub_signals_group
