# Fresh synthesis/implementation with mandatory timing signoff; no hardware access.
set foc_build_root [file normalize [file join [file dirname [info script]] .. ..]]
source [file join $foc_build_root foc.tcl]
reset_run synth_1
launch_runs synth_1 -jobs 2
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {error "FOC synthesis failed"}
launch_runs impl_1 -to_step write_bitstream -jobs 2
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {error "FOC implementation failed"}
open_run impl_1
source [file join $foc_build_root project_1 scripts foc_timing_common.tcl]
foc_timing::assert_clean
if {[llength [get_cells -quiet -hier -filter {REF_NAME =~ *vio* || REF_NAME =~ *xsdbm* || NAME =~ *dbg_hub*}]]} {
    error "Unexpected VIO/debug hub in implementation"
}
foreach foc_metric {WNS TNS WHS THS} {
    set foc_value [get_property STATS.$foc_metric [get_runs impl_1]]
    puts "FOC_PROJECT_METRIC $foc_metric=$foc_value"
    if {![string is double -strict $foc_value] || $foc_value<0} {error "FOC timing metric failed: $foc_metric"}
}
set foc_run_dir [get_property DIRECTORY [get_runs impl_1]]
set foc_output_dir [file join $foc_build_root project_1 output]
file mkdir $foc_output_dir
foreach foc_name {TOP.bit TOP_timing_signoff.rpt TOP_fault_guard_timing.rpt} {
    file copy -force [file join $foc_run_dir $foc_name] [file join $foc_output_dir $foc_name]
}
# A user rebuild replaces the supplied image: never leave its old digest beside it.
set foc_old_digest [file join $foc_output_dir SHA256.txt]
if {[file exists $foc_old_digest]} {file delete $foc_old_digest}
close_design
close_project
puts "PASS: portable project fresh build/signoff complete; output in project_1/output"
exit
