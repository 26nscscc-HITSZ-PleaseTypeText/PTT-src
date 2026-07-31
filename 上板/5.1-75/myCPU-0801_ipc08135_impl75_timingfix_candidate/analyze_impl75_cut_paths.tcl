open_checkpoint "D:/frontend/myCPU-0801_ipc08135_impl75_timingfix_candidate/reports_90m_ooc/core_top_synth.dcp"

set out_root "D:/frontend/myCPU-0801_ipc08135_impl75_timingfix_candidate/reports_90m_ooc"
set fh [open "$out_root/target_cell_inventory.txt" w]

foreach pattern {
    *head_reg*
    *req_paddr_reg*
    *mshr_line_reg*
    *mem_dc_fast*
    *s0_val_reg*
    *s1_val_reg*
    *ex_src0_reg*
    *ex_src1_reg*
    *u_regfile/rf_reg*
} {
    set objs [get_cells -quiet -hier -filter "NAME =~ $pattern"]
    puts $fh "PATTERN $pattern COUNT [llength $objs]"
    foreach obj [lrange $objs 0 19] {
        puts $fh "  $obj"
    }
}
close $fh

set head_cells [get_cells -quiet -hier -filter {NAME =~ u_rob/head_reg*}]
set req_cells  [get_cells -quiet -hier -filter {NAME =~ *req_paddr_reg*}]
set mshr_cells [get_cells -quiet -hier -filter {NAME =~ *mshr_line_reg*}]
set fast_cells [get_cells -quiet -hier -filter {NAME =~ *mem_dc_fast*}]
set rs_cells   [get_cells -quiet -hier -filter {(NAME =~ *s0_val_reg*) || (NAME =~ *s1_val_reg*) || (NAME =~ *ex_src0_reg*) || (NAME =~ *ex_src1_reg*)}]

if {[llength $head_cells] && [llength $rs_cells]} {
    report_timing -from $head_cells -to $rs_cells -delay_type max -max_paths 20 -nworst 1 \
        -file "$out_root/target_head_to_rs.rpt"
}
if {[llength $req_cells] && [llength $mshr_cells]} {
    report_timing -from $req_cells -to $mshr_cells -delay_type max -max_paths 20 -nworst 1 \
        -file "$out_root/target_req_to_mshr_line.rpt"
}
if {[llength $req_cells] && [llength $fast_cells]} {
    report_timing -from $req_cells -to $fast_cells -delay_type max -max_paths 20 -nworst 1 \
        -file "$out_root/target_req_to_fast_reg.rpt"
}
if {[llength $fast_cells] && [llength $rs_cells]} {
    report_timing -from $fast_cells -to $rs_cells -delay_type max -max_paths 20 -nworst 1 \
        -file "$out_root/target_fast_reg_to_rs.rpt"
}
exit
