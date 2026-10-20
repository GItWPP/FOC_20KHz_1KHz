# Shared timing closure/signoff for project_1.xpr and the non-project build.
# Keep the 10ns asynchronous hardware-trip budget; do not mask or relax it.
namespace eval foc_timing {
    proc guard_goal {} {
        # Physical-implementation target only. Keep the board contract at10ns.
        set_max_delay -datapath_only 8.000 -from [get_ports hard_trip_i] \
            -to [get_ports {pwm_h_o[*] pwm_l_o[*] motor_sd_a_o}]
        puts "FOC_GUARD_IMPLEMENTATION_GOAL=8.000ns SIGNOFF_BUDGET=10.000ns"
    }

    proc guard_signoff {} {
        # Restore the unchanged XDC contract before reports/checkpoints/bit.
        set_max_delay -datapath_only 10.000 -from [get_ports hard_trip_i] \
            -to [get_ports {pwm_h_o[*] pwm_l_o[*] motor_sd_a_o}]
        puts "FOC_GUARD_SIGNOFF_BUDGET_RESTORED=10.000ns"
    }

    proc has_violations {} {
        set setup [get_timing_paths -quiet -delay_type max -slack_lesser_than 0 -max_paths 1]
        set hold [get_timing_paths -quiet -delay_type min -slack_lesser_than 0 -max_paths 1]
        return [expr {[llength $setup] || [llength $hold]}]
    }

    proc assert_clean {} {
        if {![llength [get_clocks -quiet sys_clk]]} {error "FOC_SIGNOFF: missing sys_clk constraint"}
        if {![llength [get_timing_paths -quiet -delay_type max -max_paths 1]]} {
            error "FOC_SIGNOFF: no timed paths found"
        }
        if {[has_violations]} {error "FOC_SIGNOFF: setup/hold timing failed; bitstream generation blocked"}
        foreach port {pwm_h_o[0] pwm_h_o[1] pwm_h_o[2] pwm_l_o[0] pwm_l_o[1] pwm_l_o[2] motor_sd_a_o} {
            set path [get_timing_paths -quiet -from [get_ports hard_trip_i] -to [get_ports $port] -max_paths 1]
            if {[llength $path]!=1} {error "FOC_SIGNOFF: missing hardware-trip path to $port"}
            set delay [get_property DATAPATH_DELAY $path]
            set slack [get_property SLACK $path]
            puts "FOC_GUARD port=$port delay_ns=$delay slack_ns=$slack"
            if {$slack<0 || $delay>10.000} {error "FOC_SIGNOFF: hardware-trip path to $port exceeds 10ns"}
        }
        puts "PASS: FOC setup/hold and all seven 10ns hardware-trip paths meet timing"
    }

    proc close_timing {} {
        if {[has_violations]} {
            puts "FOC_TIMING_RETRY: default route failed; applying verified AggressiveExplore recovery"
            phys_opt_design -directive AggressiveExplore
            route_design -directive AggressiveExplore
        }
        assert_clean
    }
}
