# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

'''Runner for Bouffalo Lab SoCs based on the blisp tool.'''

from runners.core import ZephyrBinaryRunner, RunnerCaps


class BlispBinaryRunner(ZephyrBinaryRunner):
    '''Runner front-end for blisp.'''

    def __init__(self, cfg, chip='bl70x', port=None, baudrate=None,
                 reset=True):
        super().__init__(cfg)
        self.chip = chip
        self.port = port
        self.baudrate = baudrate
        self.reset = reset

    @classmethod
    def name(cls):
        return 'blisp'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'}, reset=True)

    @classmethod
    def do_add_parser(cls, parser):
        parser.add_argument('--chip', default='bl70x',
                            help='chip type passed to blisp')
        parser.add_argument('--port', default=None,
                            help='serial port of the ISP interface')
        parser.add_argument('--baudrate', default=None,
                            help='ISP baud rate')

    @classmethod
    def do_create(cls, cfg, args):
        return BlispBinaryRunner(cfg, chip=args.chip, port=args.port,
                                 baudrate=args.baudrate, reset=args.reset)

    def do_run(self, command, **kwargs):
        if not self.cfg.bin_file:
            raise ValueError('CONFIG_BUILD_OUTPUT_BIN must be enabled')

        self.require('blisp')

        cmd = ['blisp', 'write', f'--chip={self.chip}']

        if self.port:
            cmd.append(f'--port={self.port}')
        if self.baudrate:
            cmd.append(f'--baudrate={self.baudrate}')
        if self.reset:
            cmd.append('--reset')

        cmd.append(self.cfg.bin_file)
        self.check_call(cmd)
