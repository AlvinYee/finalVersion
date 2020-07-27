'''
Created on 22th, March, 2018

@author: Alvin Ye
'''

import sys

# add below to avoid the error of 'no module found' if running the script from .bat file
# in that case, modules are not in sys path, and can not be found automatically
sys.path.append('../')
sys.path.append('../../')
from PkdbcIterator.MddbcParser import DbcParser
from PkCanMsgLayoutComposer.MdCanMsgLayoutComposer import CanMsgLayoutComposer
from PklayoutGenerator.MdlayoutAnalyzer import SignalBitFieldGenerator


def main():
    syntax = ''
    # specify sys argv positions
    arg_dbc = 1
    arg_hfile = 2
    arg_cfile = 3
    arg_escl_name = 4
    arg_rx_msg_list = 5
    try:
        rx_msg_list = sys.argv[arg_rx_msg_list].split(",")
    except IndexError:
        rx_msg_list = None

    # create a instance of DbcParser
    my_parser = DbcParser()
    # parse dbc file and create node,msg,signal objects
    my_parser.parser_dbc(sys.argv[arg_dbc])

    with open(sys.argv[arg_hfile], 'w') as h_file:
        h_file.write('#include "mq_type.h"\n')
        with open(sys.argv[arg_cfile], 'w') as c_file:
            c_file.write('#include "I2CCAN_Par.h"\n')
            # get all ESCL Rx msgs
            try:
                all_rx_msgs = list(my_parser.can_network.nodes[sys.argv[arg_escl_name]].node_rx_msgs.values())
            except KeyError:
                print('[ERROR] please set the actual name of ESCL from dbc in .bat file')

            for msg in all_rx_msgs:
                # if Rx msg in the to-be-generated list or not specified
                if rx_msg_list is None or msg.msg_name in rx_msg_list:
                    for signal in msg.msg_signals.values():
                        # if any one signal has multiplexer, go to handle msg with multiplexer
                        if signal.signal_multiplexer is not None:
                            # msg with multiplexer will be handled as virtual_node that has several virtual msg
                            # these msg share same memory with different layout, created as union in C language
                            # in the end, each virtual msg has no multiplexer and handled as others
                            virtual_node = CanMsgLayoutComposer(msg).compose_msg_with_multiplexer()
                            for virtual_msg in virtual_node.node_tx_msgs.values():
                                h_file.write('\ntypedef struct _c_{0}_msg_typeTag\n'.format(virtual_msg.msg_name))
                                h_file.write('{\n')
                                composed_msg, sub_signals_group = CanMsgLayoutComposer(
                                    virtual_msg).compose_msg_without_multiplexer()
                                # with returned composed msg for bif field analysis
                                generator = SignalBitFieldGenerator(composed_msg)
                                generator.bit_field_generator(h_file)
                                h_file.write('}')
                                syntax += '\t_c_{0}_msg_type {1};\n'.format(virtual_msg.msg_name, virtual_msg.msg_name)
                                h_file.write('_c_{0}_msg_type;\n'.format(virtual_msg.msg_name))
                                # for a msg with multiplexer, a union instead of structure will be created
                                generator.signal_union_generator(msg, h_file)
                                # with returned sub_signals_group for signal access macro generation
                                generator.signal_access_macro_syntax(sub_signals_group, h_file)
                            h_file.write('\ntypedef union _u_{0}_msg_typeTag\n'.format(msg.msg_name))
                            h_file.write('{\n')
                            h_file.write(syntax)
                            h_file.write('}')
                            h_file.write('_u_{0}_msg_type;\n'.format(msg.msg_name))
                            c_file.write('_u_{0}_msg_type\t\t{1};\n'.format(msg.msg_name, msg.msg_name))
                            h_file.write('extern _u_{0}_msg_type\t\t{1};\n'.format(msg.msg_name, msg.msg_name))
                            break
                    # else  no signal has multiplexer
                    else:
                        h_file.write('\ntypedef struct _c_{0}_msg_typeTag\n'.format(msg.msg_name))
                        h_file.write('{\n')
                        composed_msg, sub_signals_group = CanMsgLayoutComposer(
                            msg).compose_msg_without_multiplexer()
                        # with returned composed msg for bif field analysis
                        generator = SignalBitFieldGenerator(composed_msg)
                        generator.bit_field_generator(h_file)
                        h_file.write('}')
                        h_file.write('_c_{0}_msg_type;\n'.format(msg.msg_name))
                        c_file.write('_c_{0}_msg_type\t\t{1};\n'.format(msg.msg_name, msg.msg_name))
                        h_file.write('extern _c_{0}_msg_type\t\t{1};\n'.format(msg.msg_name, msg.msg_name))
                        generator.signal_struct_generator(h_file)
                        # with returned sub_signals_group for signal access macro generation
                        generator.signal_access_macro_syntax(sub_signals_group, h_file)
    print('[INFO]: file generation succeed')


if __name__ == '__main__':
    main()
