'''
Created on 13th, March, 2018

@author: Alvin Ye
'''


class SignalBitFieldGenerator:
    '''
    this class acts to group signals by byte and create signal field inside every byte
    '''

    def __init__(self, composed_msg):
        self._composed_msg = composed_msg
        # below is a list of list, inner list is signals in one byte
        # outer list is bytes
        self._signal_groups_by_byte = [[]]
        self._syntax_type = 'uint8'
        self._syntax_struct = 'struct'
        self._syntax_def = 'typedef'
        self._syntax_delimiter = ":"
        self._syntax_ending = ';\n'
        self._syntax_define = '#define'
        self._syntax_data_type = ''
        # it binds with a dict, but with a list of tuple [(start_bit, signal)...] after sorting
        self._sorted_signals = {}

        for signal_group in self._group_signals_into_bytes():
            signal_group.reverse()

    def _group_signals_into_bytes(self):
        total_len = 0
        byte_idx = 0
        # sort signals first by start bit, this is very important
        # mainly to solve the issue for motorola format
        # after sorting signals by real start bit, gaps to the right of motorola-signal
        # has a chance to be inserted properly even sub signals of motorola-signal
        for signal in self._composed_msg.msg_signals.values():
            self._sorted_signals[signal.signal_start_bit] = signal
            #  sort signals in the dict by start bit, but return a list of tuples!
        self._sorted_signals = sorted(self._sorted_signals.items(), key=lambda d: d[0])

        # for signal in self._composed_msg.msg_signals.values():
        # new implementation
        for (start_bit, signal) in self._sorted_signals:
            total_len += signal.signal_len
            if total_len <= (byte_idx + 1) * 8:
                self._signal_groups_by_byte[byte_idx].append(signal)
            else:
                self._signal_groups_by_byte.append([])
                byte_idx += 1
                self._signal_groups_by_byte[byte_idx].append(signal)
        return self._signal_groups_by_byte

    def bit_field_generator(self, file):
        BITFIELD_STORAGE_ORDER = 'LOW_TO_HIGH'
        for signal_group in self._signal_groups_by_byte:
            if BITFIELD_STORAGE_ORDER == 'LOW_TO_HIGH':
                signal_group.reverse()
            for signal in signal_group:
                file.write(self.bit_field_syntax(signal.signal_name, signal.signal_len))

    def bit_field_syntax(self, signal_name, signal_len):
        # replace a 8-bit bit field with a byte, so that its address could bet get for a pointer
        if signal_len == 8:
            return '\t' + self._syntax_type + ' ' + signal_name + self._syntax_ending
        else:
            return ('\t' + self._syntax_type + ' ' + signal_name + self._syntax_delimiter + str(
                signal_len) + self._syntax_ending)

    def signal_union_generator(self, union_msg, file):
        for signal_group in self._signal_groups_by_byte:
            for signal in signal_group:
                if not signal.signal_name.startswith('unused'):
                    file.write(self.union_signal_macro_syntax(signal.signal_name, union_msg.msg_name,
                                                              signal.signal_msg_carrier.msg_name))

    def signal_struct_generator(self, file):
        for signal_group in self._signal_groups_by_byte:
            for signal in signal_group:
                if not signal.signal_name.startswith('unused'):
                    file.write(self.struct_signal_macro_syntax(signal.signal_name, signal.signal_msg_carrier.msg_name))

    def union_signal_macro_syntax(self, signal_name, union_msg_name, struct_name):
        return '{0} b_{1}_b\t\t({2}.{3}.{4})\n'.format(self._syntax_define, signal_name, union_msg_name,
                                                       struct_name, signal_name)

    def struct_signal_macro_syntax(self, signal_name, struct_msg_name):
        return '{0} b_{1}_b\t\t({2}.{3})\n'.format(self._syntax_define, signal_name, struct_msg_name, signal_name)

    def signal_access_macro_syntax(self, sub_signals_group, file):
        if sub_signals_group:
            for sub_signals in sub_signals_group:
                s = ''
                bit_shift = 0
                # the first element of sub signals is the original undivided signal
                if sub_signals[0].signal_little_endian:
                    # this is Intel
                    if sub_signals[0].signal_len <= 8:
                        self._syntax_data_type = 'uint8'
                    elif sub_signals[0].signal_len <= 16:
                        self._syntax_data_type = 'uint16'
                    elif sub_signals[0].signal_len <= 32:
                        self._syntax_data_type = 'uint32'
                    elif sub_signals[0].signal_len <= 64:
                        print('[INFO]: {0} - signal.length > CPU width, single access not supported'.format(
                            sub_signals[0].signal_name))
                        continue
                    else:
                        pass
                    for signal in sub_signals[1:]:
                        s += '((({0})b_{1}_b)<<{2}) | '.format(self._syntax_data_type, signal.signal_name, bit_shift)
                        bit_shift += signal.signal_len
                    else:
                        s = s.strip('| ')
                        s = '(' + s + ')\n'
                    s = '#define b_{0}_b\t'.format(sub_signals[0].signal_name) + s
                else:
                    sub_signals.reverse()
                    if sub_signals[-1].signal_len <= 8:
                        self._syntax_data_type = 'uint8'
                    elif sub_signals[-1].signal_len <= 16:
                        self._syntax_data_type = 'uint16'
                    elif sub_signals[-1].signal_len <= 32:
                        self._syntax_data_type = 'uint32'
                    elif sub_signals[-1].signal_len <= 64:
                        print('[INFO]: {0} - signal.length > CPU width, single access not supported'.format(
                            sub_signals[-1].signal_name))
                        continue
                    else:
                        pass
                    for signal in sub_signals[:-1]:
                        s += '((({0})b_{1}_b)<<{2}) | '.format(self._syntax_data_type, signal.signal_name, bit_shift)
                        bit_shift += signal.signal_len
                    else:
                        s = s.strip('| ')
                        s = '(' + s + ')\n'
                    s = '#define b_{0}_b\t'.format(sub_signals[-1].signal_name) + s
                file.write(s)
