# Project route_design POST hook. Vivado emits its default reports BEFORE this
# hook, so refresh those reports/GUI metrics after modifying the routed design.
source [file join [file dirname [info script]] foc_timing_common.tcl]
report_timing -from [get_ports hard_trip_i] -to [get_ports {pwm_h_o[*] pwm_l_o[*] motor_sd_a_o}] \
    -max_paths 7 -nworst 1 -file TOP_guard_implementation_goal.rpt
foc_timing::guard_signoff
foc_timing::close_timing
set foc_saved_impl_run [get_param project.isImplRun]
set_param project.isImplRun true
set foc_report_rc [catch {
    generate_parallel_reports -reports {
        "report_drc -file TOP_drc_routed.rpt -pb TOP_drc_routed.pb -rpx TOP_drc_routed.rpx"
        "report_methodology -file TOP_methodology_drc_routed.rpt -pb TOP_methodology_drc_routed.pb -rpx TOP_methodology_drc_routed.rpx"
        "report_power -file TOP_power_routed.rpt -pb TOP_power_summary_routed.pb -rpx TOP_power_routed.rpx"
        "report_route_status -file TOP_route_status.rpt -pb TOP_route_status.pb"
        "report_timing_summary -max_paths 10 -report_unconstrained -file TOP_timing_summary_routed.rpt -pb TOP_timing_summary_routed.pb -rpx TOP_timing_summary_routed.rpx -warn_on_violation"
        "report_incremental_reuse -file TOP_incremental_reuse_routed.rpt"
        "report_clock_utilization -file TOP_clock_utilization_routed.rpt"
        "report_bus_skew -warn_on_violation -file TOP_bus_skew_routed.rpt -pb TOP_bus_skew_routed.pb -rpx TOP_bus_skew_routed.rpx"
    }
} foc_report_result foc_report_options]
set_param project.isImplRun $foc_saved_impl_run
if {$foc_report_rc} {return -options $foc_report_options $foc_report_result}
report_timing_summary -delay_type min_max -max_paths 10 -file TOP_timing_signoff.rpt
report_timing -from [get_ports hard_trip_i] -to [get_ports {pwm_h_o[*] pwm_l_o[*] motor_sd_a_o}] \
    -max_paths 7 -nworst 1 -file TOP_fault_guard_timing.rpt
