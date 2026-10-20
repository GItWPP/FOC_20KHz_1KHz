# Source in an opened project, including the current Vivado GUI Tcl console.
set foc_timing_script_dir [file normalize [file dirname [info script]]]
set foc_impl_run [get_runs -quiet impl_1]
if {[llength $foc_impl_run]!=1} {error "Open project_1.xpr before configuring FOC implementation timing"}
foreach foc_hook_name {pre_place_foc.tcl post_route_foc.tcl pre_bitstream_foc.tcl foc_timing_common.tcl} {
    set foc_hook_path [file join $foc_timing_script_dir $foc_hook_name]
    if {![llength [get_files -quiet $foc_hook_path]]} {
        add_files -fileset utils_1 -norecurse $foc_hook_path
    }
}
# Route the 10ns asynchronous output guards aggressively on the first pass.
# This also avoids relying on post-route physical optimization for those nets.
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE AggressiveExplore $foc_impl_run
set_property STEPS.PLACE_DESIGN.TCL.PRE [file join $foc_timing_script_dir pre_place_foc.tcl] $foc_impl_run
set_property STEPS.ROUTE_DESIGN.TCL.POST [file join $foc_timing_script_dir post_route_foc.tcl] $foc_impl_run
set_property STEPS.WRITE_BITSTREAM.TCL.PRE [file join $foc_timing_script_dir pre_bitstream_foc.tcl] $foc_impl_run
puts "FOC project timing configured: 8ns guard implementation goal, original10ns signoff, AggressiveExplore route and mandatory pre-bitstream checks"
