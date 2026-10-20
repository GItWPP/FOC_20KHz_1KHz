# Project write_bitstream PRE hook: generated bit files must meet the same signoff.
source [file join [file dirname [info script]] foc_timing_common.tcl]
foc_timing::assert_clean
