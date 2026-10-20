# Batch project creation only; no synthesis and no hardware access.
source [file normalize [file join [file dirname [info script]] .. .. foc.tcl]]
close_project
exit
