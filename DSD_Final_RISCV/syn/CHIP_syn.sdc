#You may modified the clock constraints 
#or add more constraints for your design
####################################################
set cycle  3.1        
####################################################


#The following are design spec. for synthesis
#You can NOT modify this seciton 
#####################################################
create_clock -name CLK -period $cycle [get_ports clk]
set_fix_hold                          [get_clocks CLK]
set_dont_touch_network                [get_clocks CLK]
set_ideal_network                     [get_ports clk]
set_clock_uncertainty            0.1  [get_clocks CLK] 
set_clock_latency                0.5  [get_clocks CLK] 

set_max_fanout 6 [all_inputs] 

set_operating_conditions -min_library fast -min fast -max_library slow -max slow
set_wire_load_model -name tsmc13_wl10 -library slow  
set_drive        1     [all_inputs]
set_load         1     [all_outputs]
#####################################################

#The following are input/output delay constraints
#You NEED to modify the constraints to pass gate-level simulation correctly (check TB and slow_memory)
#You may also add more constraints for your design (but do not overwrite the existing ones in above section)
#####################################################
set t_long [expr {$cycle / 2.0 + 0.1}]
echo "t_long = $t_long"
set t_short  0.1
set_input_delay  $t_long  -clock CLK [remove_from_collection [remove_from_collection [all_inputs] [get_ports clk]] [get_ports rst_n]]
set_input_delay  $t_short -clock CLK [get_ports rst_n]
set_output_delay $t_long  -clock CLK [remove_from_collection [remove_from_collection [remove_from_collection [remove_from_collection [all_outputs] [get_ports DCACHE_addr]] [get_ports DCACHE_wdata]] [get_ports DCACHE_wen]] [get_ports PC]]
set_output_delay $t_short -clock CLK [get_ports DCACHE_addr]
set_output_delay $t_short -clock CLK [get_ports DCACHE_wdata]
set_output_delay $t_short -clock CLK [get_ports DCACHE_wen]
set_output_delay $t_short -clock CLK [get_ports PC]
#####################################################
