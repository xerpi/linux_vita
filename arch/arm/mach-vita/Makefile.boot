# SPDX-License-Identifier: GPL-2.0

# The Vita's shared (NS) DRAM base is actually at 0x40200000
# but __fixup_pv_table expects it to be 16MiB aligned
zreladdr-y     += 0x41008000
params_phys-y  := 0x41000100
initrd_phys-y  := 0x41000000
