# SPDX-License-Identifier: Apache-2.0

board_runner_args(pylink "--device=EFR32MG1PxxxF256")
board_runner_args(jlink "--device=EFR32MG1PxxxF256")
board_runner_args(openocd)

# pylink flashes through the J-Link SDK library directly, so no JLinkExe
# installation is needed. Register it before jlink so it becomes the default
# flasher; debug still uses gdb via the jlink runner.
board_finalize_runner_args(pylink)
board_set_flasher_ifnset(pylink)

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)