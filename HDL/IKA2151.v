module IKA2151
(
    //chip clock
    input   wire            i_EMUCLK, //emulator master clock
    input   wire            i_phiM_PCEN_n, //phiM clock enable

    //chip reset
    input   wire            i_IC_n,    

    //phi1
    output  wire            o_phi1,

    //sh
    output  wire            o_SH1,
    output  wire            o_SH2
);


///////////////////////////////////////////////////////////
//////  Clock and reset
////

wire            phi1pcen_n, phi1ncen_n;
wire            mrst_n;



///////////////////////////////////////////////////////////
//////  Interconnects
////

//timings
wire            cycle_12_28, cycle_05_21_n, cycle_byte;

//LFO
wire    [7:0]   LFA, LFP;





mdl_timinggen TIMINGGEN (
    .i_EMUCLK                   (i_EMUCLK                   ),
    .i_phiM_PCEN_n              (i_phiM_PCEN_n              ),

    .i_IC_n                     (i_IC_n                     ),

    .o_MRST_n                   (mrst_n                     ),

    .o_phi1                     (o_phi1                     ),
    .o_phi1_PCEN_n              (phi1pcen_n                 ),
    .o_phi1_NCEN_n              (phi1ncen_n                 ),

    .o_SH1                      (o_SH1                      ),
    .o_SH2                      (o_SH2                      ),

    .o_CYCLE_12_28              (cycle_12_28                ),
    .o_CYCLE_05_21_n            (cycle_05_21_n              ),
    .o_CYCLE_BYTE               (cycle_byte                 )
);


mdl_lfo LFO (
    .i_EMUCLK                   (i_EMUCLK                   ),

    .i_MRST_n                   (mrst_n                     ),
    
    .i_phi1_PCEN_n              (phi1pcen_n                 ),
    .i_phi1_NCEN_n              (phi1ncen_n                 ),
    
    .i_CYCLE_12_28              (cycle_12_28                ),
    .i_CYCLE_05_21_n            (cycle_05_21_n              ),
    .i_CYCLE_BYTE               (cycle_byte                 ),
    
    .i_LFRQ                     (8'hF2                      ),
    .i_AMD                      (7'hEF                      ),
    .i_PMD                      (7'hEF                      ),
    .i_W                        (2'h2                       ),
    .i_TEST                     (8'h00                      ),
    
    .i_LFRQ_UPDATE_n            (1'b1                       ),

    .o_LFA                      (                           ),
    .o_LFP                      (LFP                        )
);


mdl_pg PG (
    .i_EMUCLK                   (i_EMUCLK                   ),

    .i_MRST_n                   (mrst_n                     ),
    
    .i_phi1_PCEN_n              (phi1pcen_n                 ),
    .i_phi1_NCEN_n              (phi1ncen_n                 ),

    .i_KC                       (7'h3A                      ),
    .i_KF                       (6'h2B                      ),
    .i_PMS                      (3'd7                       ),
    .i_DT2                      (2'd3                       ),
    .i_LFP                      (LFP                        )
);

endmodule