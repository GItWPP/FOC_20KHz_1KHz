# Program volatile FPGA configuration only; UART starts disabled after loading.
# vivado -mode batch -source program_foc.tcl -tclargs <optional TOP.bit directory>
set task_root [file normalize [file join [file dirname [info script]] .. ..]]
set task_image_dir [expr {[llength $argv] ? [file normalize [lindex $argv 0]] : [file join $task_root project_1 output]}]
if {![file exists [file join $task_image_dir TOP.bit]]} {error "TOP.bit not found: $task_image_dir"}
open_hw_manager
set task_open 0
set task_code [catch {
    connect_hw_server -url localhost:3121 -allow_non_jtag
    set task_target [get_hw_targets localhost:3121/xilinx_tcf/Digilent/210512180081]
    if {[llength $task_target]!=1} {error "Expected JTAG adapter210512180081 not found"}
    current_hw_target $task_target
    set_property PARAM.FREQUENCY 1000000 $task_target
    open_hw_target
    set task_open 1
    set task_device [get_hw_devices xc7z020_1]
    current_hw_device $task_device
    # Clear cached probes from the previous VIO image. This design has no LTX.
    set_property PROBES.FILE {} $task_device
    set_property FULL_PROBES.FILE {} $task_device
    set_property PROGRAM.FILE [file join $task_image_dir TOP.bit] $task_device
    program_hw_devices $task_device
    refresh_hw_device -update_hw_probes false $task_device
    puts "PASS: UART-only volatile FPGA programmed; no debug probes; start defaults to0"
} task_message task_options]
if {$task_open} {catch {close_hw_target}}
catch {disconnect_hw_server}
close_hw_manager
if {$task_code} {return -options $task_options $task_message}
exit
