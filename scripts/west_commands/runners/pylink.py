# Copyright The Zephyr Project Contributors
#
# SPDX-License-Identifier: Apache-2.0

'''Runner for flashing a SEGGER J-Link target through pylink.

This talks to the J-Link SDK shared library (libjlinkarm) directly via the
``pylink`` package, so no JLinkExe / J-Link Commander installation is required:
only the self-contained shared library needs to be present on the machine.
'''

import os

from runners.core import RunnerCaps, ZephyrBinaryRunner

try:
    import pylink
    MISSING_REQUIREMENTS = False
except ImportError:
    MISSING_REQUIREMENTS = True


class PylinkRunner(ZephyrBinaryRunner):
    '''Runner front-end for flashing through pylink.'''

    def __init__(self, cfg, device, dev_id=None, speed=4000, iface='swd',
                 dt_flash=False, reset=True, lib=None):
        super().__init__(cfg)

        self.device = device
        self.dev_id = dev_id
        self.speed = speed
        self.iface = iface
        self.dt_flash = dt_flash
        self.reset = reset
        self.lib = lib

        self.hex_name = cfg.hex_file
        self.bin_name = cfg.bin_file
        self.elf_name = cfg.elf_file

    @classmethod
    def name(cls):
        return 'pylink'

    @classmethod
    def capabilities(cls):
        return RunnerCaps(commands={'flash'},
                          dev_id=True,
                          reset=True,
                          flash_addr=True,
                          file=True)

    @classmethod
    def do_add_parser(cls, parser):
        parser.add_argument('--device', required=True,
                            help='device identifier, e.g. EFR32MG1PxxxF256')
        parser.add_argument('--speed', type=int, default=4000,
                            help='SWD clock in kHz (default 4000)')
        parser.add_argument('--iface', choices=['swd', 'jtag'], default='swd',
                            help='debug protocol to use (default swd)')
        parser.add_argument('--pylink-lib',
                            help='path to the libjlinkarm shared library '
                                 '(auto-detected if omitted)')

    @classmethod
    def dev_id_help(cls) -> str:
        return 'J-Link serial number (omit to auto-select the first probe)'

    @classmethod
    def do_create(cls, cfg, args):
        return PylinkRunner(cfg, device=args.device,
                            dev_id=args.dev_id,
                            speed=args.speed,
                            iface=args.iface,
                            dt_flash=args.dt_flash,
                            reset=args.reset,
                            lib=args.pylink_lib)

    def do_run(self, command, **kwargs):
        if MISSING_REQUIREMENTS:
            raise RuntimeError('pylink is not installed; '
                               'install it with "pip install pylink-square"')
        if command == 'flash':
            self.do_flash(**kwargs)
        else:
            raise ValueError(f'unsupported command: {command}')

    def do_flash(self, **kwargs):
        image, addr = self._pick_image()
        self.logger.info(f'flashing {image}{f" at 0x{addr:x}" if image.endswith(".bin") else ""}')

        lib = pylink.Library(dllpath=self.lib) if self.lib else pylink.Library()
        jlink = pylink.JLink(lib=lib)
        try:
            jlink.open(serial_no=self.dev_id)
            jlink.set_tif(pylink.enums.JLinkInterfaces.SWD if self.iface == 'swd'
                          else pylink.enums.JLinkInterfaces.JTAG)
            jlink.connect(self.device, speed=self.speed)
            jlink.reset(halt=True)

            jlink.flash_file(image, addr,
                             on_progress=lambda action, pstr, pct:
                             self.logger.debug(f'{action}: {pct}%'))

            if self.reset:
                jlink.reset(halt=False)
            else:
                self.logger.info('target left halted (--no-reset)')
        finally:
            jlink.close()

    def _pick_image(self):
        # Prefer .hex (addresses are embedded), then .bin (needs flash address),
        # then .elf.
        if self.hex_name is not None:
            return self.hex_name, 0x0
        if self.bin_name is not None:
            addr = (self.flash_address_from_build_conf(self.build_conf)
                    if self.dt_flash else 0x0)
            return self.bin_name, addr
        if self.elf_name is not None:
            return self.elf_name, 0x0
        raise ValueError('no hex, bin or elf image found for the build')