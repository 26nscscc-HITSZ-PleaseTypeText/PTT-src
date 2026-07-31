set cpu_root "D:/frontend/myCPU-0801_ipc08135_impl75_timingfix_candidate"
set out_root "D:/frontend/myCPU-0801_ipc08135_impl75_timingfix_candidate/reports_90m_ooc"
set xdc_file "D:/frontend/myCPU-0731_ipc09_90m_candidate/reports_90m_ooc/cpu_90m.xdc"
file mkdir $out_root
set_param general.maxThreads 1
set_param synth.maxThreads 1
set rtl_files [glob -nocomplain -directory $cpu_root *.v]
foreach rtl_dir [glob -nocomplain -types d -directory $cpu_root *] {
    set rtl_files [concat $rtl_files [glob -nocomplain -directory $rtl_dir *.v]]
    foreach rtl_subdir [glob -nocomplain -types d -directory $rtl_dir *] {
        set rtl_files [concat $rtl_files [glob -nocomplain -directory $rtl_subdir *.v]]
    }
}
set_property include_dirs [list $cpu_root] [current_fileset]
read_verilog -sv $rtl_files
read_xdc $xdc_file
synth_design -top core_top -part xc7a200tfbg676-2 -mode out_of_context \
    -flatten_hierarchy rebuilt
report_utilization -file "$out_root/utilization_synth.rpt"
report_timing_summary -delay_type max -max_paths 100 \
    -file "$out_root/timing_synth.rpt"
report_timing -delay_type max -max_paths 500 -nworst 20 \
    -path_type full_clock_expanded -file "$out_root/timing_paths.rpt"
write_checkpoint -force "$out_root/core_top_synth.dcp"
exit
