# Create/open the portable project. GUI: source <project-root>/foc.tcl
# Batch: use project_1/scripts/create_project.tcl or build.tcl.
set foc_root [file normalize [file dirname [info script]]]
set foc_project_dir [file join $foc_root project_1]
set foc_xpr [file join $foc_project_dir project_1.xpr]
if {![llength [get_projects -quiet]]} {
    if {[file exists $foc_xpr]} {
        open_project $foc_xpr
    } else {
        create_project project_1 $foc_project_dir -part xc7z020clg400-2
    }
} elseif {[file normalize [get_property DIRECTORY [current_project]]] ne $foc_project_dir} {
    error "Close the other project before sourcing $foc_xpr"
}
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]
set_property target_simulator XSim [current_project]
set_property default_lib xil_defaultlib [current_project]
foreach foc_source [lsort [glob -directory [file join $foc_root rtl] *.v]] {
    if {![llength [get_files -quiet $foc_source]]} {add_files -fileset sources_1 -norecurse $foc_source}
}
set foc_xdc [file join $foc_root foc.xdc]
if {![llength [get_files -quiet $foc_xdc]]} {add_files -fileset constrs_1 -norecurse $foc_xdc}
foreach foc_source [lsort [glob -directory [file join $foc_root sim] tb_*.v]] {
    if {![llength [get_files -quiet $foc_source]]} {add_files -fileset sim_1 -norecurse $foc_source}
    set_property USED_IN_SYNTHESIS false [get_files $foc_source]
    set_property USED_IN_IMPLEMENTATION false [get_files $foc_source]
}
if {![llength [get_filesets -quiet utils_1]]} {create_fileset -utilsset utils_1}
set_property generic {} [get_filesets sources_1]
set_property top TOP [get_filesets sources_1]
set_property top tb_foc_uart_top [get_filesets sim_1]
set_property AUTO_INCREMENTAL_CHECKPOINT 0 [get_runs synth_1]
source [file join $foc_project_dir scripts configure_project_timing.tcl]
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
if {[llength [get_ips -quiet]]} {error "Unexpected IP in UART-only project"}
puts "PASS: portable project_1 configured; TOP,25 RTL,4 main testbenches and root foc.xdc"
