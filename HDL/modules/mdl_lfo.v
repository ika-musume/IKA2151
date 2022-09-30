module mdl_lfo
(
    //master clock
    input   wire            i_EMUCLK, //emulator master clock

    //core internal reset
    input   wire            i_MRST_n,

    //internal clock
    input   wire            i_phi1_PCEN_n, //positive edge clock enable for emulation
    input   wire            i_phi1_NCEN_n, //engative edge clock enable for emulation

    //timings
    input   wire            i_CYCLE_12_28,
    input   wire            i_CYCLE_05_21_n,
    input   wire            i_CYCLE_BYTE,

    //register data
    input   wire    [7:0]   i_LFRQ , //LFO frequency
    input   wire    [6:0]   i_AMD, //amplitude modulation depth
    input   wire    [6:0]   i_PMD, //phase modulation depth
    input   wire    [1:0]   i_W, //waveform select
    input   wire    [7:0]   i_TEST, //test register

    //control signal
    input   wire            i_LFRQ_UPDATE_n,

    output  wire    [7:0]   o_LFP,
    output  wire    [7:0]   o_LFA
);


///////////////////////////////////////////////////////////
//////  Clock and reset
////

wire            phi1pcen_n = i_phi1_PCEN_n;
wire            phi1ncen_n = i_phi1_NCEN_n;
wire            mrst_n = i_MRST_n;



///////////////////////////////////////////////////////////
//////  Cycle number
////

//additional cycle bits
reg             cycle_06_22, cycle_13_29, cycle_14_30, cycle_15_31;
reg             debug_cycle_07_23;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        cycle_06_22 <= ~i_CYCLE_05_21_n;

        cycle_13_29 <= i_CYCLE_12_28;
        cycle_14_30 <= cycle_13_29;
        cycle_15_31 <= cycle_14_30;

        debug_cycle_07_23 <= cycle_06_22;
    end
end



///////////////////////////////////////////////////////////
//////  Prescaler
////

//counter
reg     [3:0]   prescaler_cntr;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        if(!mrst_n) prescaler_cntr <= 4'd0; //counter reset
        else begin
            if(i_CYCLE_12_28) begin
                if(prescaler_cntr == 4'd15) prescaler_cntr <= 4'd0; //wrap
                else prescaler_cntr <= prescaler_cntr + 4'd1; //count up
            end
        end
    end
end

//cycle 2
reg             prescaler_cycle_2;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) prescaler_cycle_2 <= prescaler_cntr[3:0] == 4'd2;
end

//counter carry
wire            prescaler_cout = (prescaler_cntr == 4'd15) & i_CYCLE_12_28;



///////////////////////////////////////////////////////////
//////  LFO LUT and output latch
////

/*
    pre-initialized LFO LUT. The bit order of the original chip is:
      (LEFT) D1 - D7 / D14 - D8 (RIGHT)
    D0 is controlled by row F enable signal, so the table below contains the precalculated values

    lfo dout latch:
    the original one uses different edges due to carry delay of the counter cells
    so the counter for MSBs is delayed by a half phi1.
*/

reg     [14:0]  lfolut_dout;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        case(i_LFRQ[7:4])
            4'hF: lfolut_dout <= 15'h7FFF;
            4'hE: lfolut_dout <= 15'h7FFE;
            4'hD: lfolut_dout <= 15'h7FFC;
            4'hC: lfolut_dout <= 15'h7FF8;
            4'hB: lfolut_dout <= 15'h7FF0;
            4'hA: lfolut_dout <= 15'h7FE0;
            4'h9: lfolut_dout <= 15'h7FC0;
            4'h8: lfolut_dout <= 15'h7F80;
            4'h7: lfolut_dout <= 15'h7F00;
            4'h6: lfolut_dout <= 15'h7E00;
            4'h5: lfolut_dout <= 15'h7C00;
            4'h4: lfolut_dout <= 15'h7800;
            4'h3: lfolut_dout <= 15'h7000;
            4'h2: lfolut_dout <= 15'h6000;
            4'h1: lfolut_dout <= 15'h4000;
            4'h0: lfolut_dout <= 15'h1000;
        endcase
    end
end



///////////////////////////////////////////////////////////
//////  LFRQ hi bits counter
////

//define hicntr register
reg     [14:0]  hicntr;
reg             hicntr_cnt;
wire            hicntr_cout = (hicntr == 15'h7FFF) & hicntr_cnt;

//hicntr cnt up signal
reg             prescaler_cout_dlyd1;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        prescaler_cout_dlyd1 <= prescaler_cout;
        hicntr_cnt <= prescaler_cout_dlyd1 | i_TEST[3]; //de morgan
    end
end

//hicntr preload signal
reg             hicntr_cout_dlyd1, freq_update;
reg             hicntr_ld;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        hicntr_cout_dlyd1 <= hicntr_cout;
        freq_update <= ~i_LFRQ_UPDATE_n;

        hicntr_ld <= (hicntr_cout_dlyd1 | freq_update);
    end
end

//counter
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        if(!mrst_n) hicntr <= 15'hFFFF; //counter reset originally 0!!!!
        else begin
            if(hicntr_ld) hicntr <= lfolut_dout; //counter load
            else begin
                if(hicntr_cnt) begin
                    if(hicntr == 15'h7FFF) hicntr <= 15'h0000; //wrap
                    else hicntr <= hicntr + 15'h0001; //count up
                end
            end
        end
    end
end



///////////////////////////////////////////////////////////
//////  LFRQ lo bits counter
////

//async code for test
/*
    reg cycle14_platch, cycle14_nlatch;
    wire ddl1_en = ~(~cycle14_platch | cycle14_nlatch);
    always @(posedge i_EMUCLK) begin
        if(!phi1pcen_n) cycle14_platch <= cycle_14_30;
        if(!phi1ncen_n) cycle14_nlatch <= ~cycle14_platch;
    end

    reg ddl2_en;
    always @(posedge i_EMUCLK) begin
        if(!phi1pcen_n) ddl2_en <= ~i_CYCLE_05_21_n;
    end

    reg ddl1, ddl2;
    always @(*) begin
        if(ddl1_en) ddl1 <= ~hicntr_cout_dlyd1;
        if(ddl2_en) ddl2 <= ~ddl1;
    end
*/

//hicntr cout delay
reg             hicntr_cout_step1, hicntr_cout_step2;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        if(cycle_14_30) hicntr_cout_step1 <= hicntr_cout; //not dlyd1! async latch elimination
    end
    if(!phi1pcen_n) begin //use positive edge
        if(!i_CYCLE_05_21_n) hicntr_cout_step2 <= hicntr_cout_step1;
    end
end

//locntr cnt up and output decoder enable
reg             locntr_cnt;
wire            locntr_decode_en = (cycle_13_29 & hicntr_cout_step2); //de morgan
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        locntr_cnt <= locntr_decode_en;
    end
end

//counter
reg     [3:0]   locntr;
always @(posedge i_EMUCLK) begin
    if(!phi1pcen_n) begin //use positive edge
        if(!mrst_n) locntr <= 4'h0; //counter reset
        else begin
            if(locntr_cnt) begin
                if(locntr == 4'hF) locntr <= 4'h0; //wrap
                else locntr <= locntr + 4'h1; //count up
            end
        end
    end
end

//locntr complete flag
reg             locntr_complete; //use positive edge
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        if(locntr_decode_en) begin //decode locntr value only when decode_en == 1
            casez(locntr)
                4'b???0: locntr_complete <= i_LFRQ[3];
                4'b??01: locntr_complete <= i_LFRQ[2];
                4'b?011: locntr_complete <= i_LFRQ[1];
                4'b0111: locntr_complete <= i_LFRQ[0];
            endcase
        end
        else locntr_complete <= 1'b0; //disable
    end
end



///////////////////////////////////////////////////////////
//////  LFO clock generation
////

reg             lfo_clk;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        lfo_clk <= |{hicntr_cout, locntr_complete, i_TEST[2]};
    end
end



///////////////////////////////////////////////////////////
//////  latched LFO clock
////

//The original one used dynamic D-latch to latch lfo_clk
//I reused the signal above to eliminate a latch.
reg             lfo_clk_locntr_latched = 1'b0; //dynamic d latch
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        if(cycle_14_30) lfo_clk_locntr_latched <= |{hicntr_cout, locntr_complete, i_TEST[2]};
    end
end



///////////////////////////////////////////////////////////
//////  waveform decoder
////

reg     [1:0]   wfsel;
wire            wfsel_noise = (wfsel == 2'd3);
wire            wfsel_tri   = (wfsel == 2'd2);
wire            wfsel_sq    = (wfsel == 2'd1);
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        wfsel <= i_W;
    end
end



///////////////////////////////////////////////////////////
//////  LFO phase accumulator
////

//test bit 1 latch
reg             tst_bit1_latched;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) tst_bit1_latched <= i_TEST[1];
end

//phase accumulator
reg     [15:0]  phase_acc; //phase accumulator shift register
wire            phase_acc_lsb = phase_acc[0];

//full adder
wire    [1:0]   phase_acc_fa;
reg             phase_acc_fa_prev_carry = 1'b0;
always @(posedge i_EMUCLK or negedge mrst_n) begin //async reset
    if(!mrst_n) phase_acc_fa_prev_carry <= 1'b0;
    else begin
        if(!phi1ncen_n) phase_acc_fa_prev_carry <= phase_acc_fa[1]; //store previous carry(serial full adder)
    end
end

wire            phase_acc_fa_a = &{&{cycle_15_31, lfo_clk, ~wfsel_noise},
                                    wfsel_tri};
wire            phase_acc_fa_b = &{mrst_n,
                                   phase_acc_lsb,
                                   ~(lfo_clk_locntr_latched & wfsel_noise),
                                   ~tst_bit1_latched};
wire            phase_acc_fa_cin = ~(|{cycle_15_31, ~phase_acc_fa_prev_carry, wfsel_noise} &
                                     ~&{cycle_15_31, lfo_clk, ~wfsel_noise});

assign  phase_acc_fa = phase_acc_fa_a + phase_acc_fa_b + phase_acc_fa_cin;


//noise input
reg             lfsr_lsb_latched, noise_stream;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        lfsr_lsb_latched <= 1'b0;
        noise_stream <= lfsr_lsb_latched & noise_stream; //de morgan
    end
end

//phase accumulator input
wire            phase_acc_input = phase_acc_fa[0] | (wfsel_noise & noise_stream);

//shift accumulator
always @(posedge i_EMUCLK or negedge mrst_n) begin //async reset
    if(!mrst_n) phase_acc <= 16'h0;
    else begin
        if(!phi1ncen_n) begin
            phase_acc[15] <= phase_acc_input;
            phase_acc[14:0] <= phase_acc[15:1];
        end
    end
end

//for debug
reg     [15:0]      phase_acc_debug;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        if(cycle_15_31) phase_acc_debug <= phase_acc;
    end
end



///////////////////////////////////////////////////////////
//////  Bit select counter for serial integration
////

//this counter select AMD/PMD bit
wire            bitcntr_rst = prescaler_cycle_2 & i_CYCLE_12_28;
reg     [3:0]   bitcntr; //counter
reg     [2:0]   bitsel; //bit select counter value latch
always @(posedge i_EMUCLK or negedge mrst_n) begin
    if(!mrst_n) begin
        bitcntr <= 4'h0;
        bitsel <= 3'h0;
    end
    else begin
        if(!phi1pcen_n) begin //count at positive edge
            if(bitcntr_rst) bitcntr <= 4'h0; //reset
            else begin
                if(cycle_14_30) begin
                    if(bitcntr == 4'hF) bitcntr <= 4'h0; //wrap
                    else bitcntr <= bitcntr + 4'h1;
                end
            end
        end
    end

    if(!phi1ncen_n) bitsel <= bitcntr[2:0]; //store value at negative edge
end

//timings/control
wire            a_np_sel = ~bitcntr[3]; //AMD/PMD mux select
wire            bitcntr_0_8 = (bitcntr[2:0] == 3'h0);



///////////////////////////////////////////////////////////
//////  Oscillator base value generator
////

//triangle/sawtooth sign bit latch
/*
    phi1    |_______|¯¯¯¯¯¯¯|_______|¯¯¯¯¯¯¯|_______|¯¯¯¯¯¯¯
            |----(14_30)----|----(15_31)----|----(0_16)----|

    d valid <--------------> <-------------> <------------->
    latchen ________________|¯¯¯¯¯¯¯|_______________________
    dff                             ^ <-- sample here     

*/
reg             wf_tri_sign, wf_saw_sign;
always @(posedge i_EMUCLK) begin //use posedge
    if(!phi1pcen_n) begin
        if(cycle_15_31) begin
            wf_tri_sign <= phase_acc[8];
            wf_saw_sign <= phase_acc[7];
        end
    end
end

//amd/pmd select latch
reg             a_np_sel_latched;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        a_np_sel_latched <= a_np_sel;
    end
end

//base value stream, behavioral implementation
//              waveform type                                     (             PM             )  (             AM              )
wire            noise_value_stream = (a_np_sel_latched == 1'b0) ? (phase_acc_fa_b ^ wf_saw_sign) : phase_acc_fa_b ^ 1'b1        ;
wire            tri_value_stream   = (a_np_sel_latched == 1'b0) ? (phase_acc_fa_b ^ wf_saw_sign) : phase_acc_fa_b ^ ~wf_tri_sign;
wire            sq_value_stream    = (a_np_sel_latched == 1'b0) ?          cycle_06_22           :         ~wf_saw_sign         ;
wire            saw_value_stream   = (a_np_sel_latched == 1'b0) ? (phase_acc_fa_b ^ wf_saw_sign) : phase_acc_fa_b ^ 1'b1        ;

//input selector
reg             base_value_input;
always @(*) begin
    if(i_CYCLE_BYTE) begin
        case(wfsel)
            2'd3: base_value_input <= noise_value_stream;
            2'd2: base_value_input <= tri_value_stream; //gawr gura
            2'd1: base_value_input <= sq_value_stream;
            2'd0: base_value_input <= saw_value_stream;
        endcase
    end
    else base_value_input <= 1'b0;
end

//base value shift register
reg     [6:0]   base_value_sr;
always @(posedge i_EMUCLK or negedge mrst_n) begin
    if(!mrst_n) base_value_sr <= 7'h00;
    else begin
        if(!phi1ncen_n) begin
            base_value_sr[6] <= base_value_input;
            base_value_sr[5:0] <= base_value_sr[6:1];
        end
    end
end

//debug
reg     [6:0]   base_value_am_debug, base_value_pm_debug;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        if(debug_cycle_07_23) begin
            if(a_np_sel_latched) base_value_am_debug <= base_value_sr;
            else base_value_pm_debug <= base_value_sr;
        end
    end
end



///////////////////////////////////////////////////////////
//////  Volume multiplier
////

/*
                         TAP
                                MSB 6   5   4   3   2   1   0 LSB

    Base value shift register       0   1   1   0   1   0   1

    Volume register(AMD/PMD)        1   0   0   1   1   0   0

    1. pick volume_reg[6] and do AND with the base value[6:0], serially.
    -> 0110101

    2. pick volume reg[5] and do AND with the base value {1'b0, value[6:1]}, serially,
    -> 0000000

    3. pick volume reg[4] and do AND with the base value {2'b00, value[6:2]}, serially,
    -> 0000000

    ...repeat for all bits of AMD/PMD

    now sum the values below
        0110101
        0000000
        0000000
        0000110
        0000011
        0000000
        0000000 +
    =   0111110       ====> this's the final value calculated

    Volume format X.XXXXXX fixed point.
    AMD/PMD bit 6 is decimal part, and bit 5 to 0 is fractional part. Step width 0.015625.
*/

//AMD/PMD mux
reg     [6:0]   ap_muxed;
always @(i_EMUCLK) begin
    ap_muxed <= (a_np_sel == 1'b1) ? i_AMD : i_PMD;
end

//bit selector
reg             multiplier_fa_b;
always @(*) begin
    case(bitsel)
        3'b000: multiplier_fa_b <= base_value_sr[0] & ap_muxed[6];
        3'b001: multiplier_fa_b <= base_value_sr[1] & ap_muxed[5];
        3'b010: multiplier_fa_b <= base_value_sr[2] & ap_muxed[4];
        3'b011: multiplier_fa_b <= base_value_sr[3] & ap_muxed[3];
        3'b100: multiplier_fa_b <= base_value_sr[4] & ap_muxed[2];
        3'b101: multiplier_fa_b <= base_value_sr[5] & ap_muxed[1];
        3'b110: multiplier_fa_b <= base_value_sr[6] & ap_muxed[0];
        3'b111: multiplier_fa_b <= 1'b0;
    endcase
end

wire            bitsel_0 = (bitsel == 3'b000);
wire            bitsel_7 = (bitsel == 3'b111);

//multiplier
wire    [1:0]   multiplier_fa;
reg     [15:0]  multiplier_sr = 16'h0;

always @(posedge i_EMUCLK or negedge mrst_n) begin
    if(!mrst_n) multiplier_sr <= 16'h0; //reset
    else begin
        if(!phi1ncen_n) begin
            multiplier_sr[15] <= multiplier_fa[0];
            multiplier_sr[14:0] <= multiplier_sr[15:1];
        end
    end
end

reg             multiplier_prev_carry = 1'b0;
always @(posedge i_EMUCLK or negedge mrst_n) begin
    if(!mrst_n) multiplier_prev_carry <= 1'b0; //reset
    else begin
        if(!phi1ncen_n) multiplier_prev_carry <= multiplier_fa[1];
    end
end

wire            multiplier_fa_a = ~(~multiplier_sr[0] | bitsel_0);
wire            multiplier_fa_cin = multiplier_prev_carry & ~cycle_15_31;

assign  multiplier_fa = multiplier_fa_a + multiplier_fa_b + multiplier_fa_cin;



///////////////////////////////////////////////////////////
//////  LFA/LFP latch
////

//latch enables
reg             bitcntr_0_8_dlyd;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) bitcntr_0_8_dlyd <= bitcntr_0_8;
end

//LFA LFP register load
wire            lfa_reg_ld = &{cycle_15_31, bitsel_7, a_np_sel};
wire            lfp_reg_ld = &{cycle_15_31, bitsel_7, ~a_np_sel};

//LFA LFP register
reg     [7:0]   lfa_reg, lfp_reg;
assign  o_LFA = lfa_reg;
assign  o_LFP = lfp_reg;

//LFP sign/value control
wire            pmd_zero = (i_PMD == 7'h00);
wire            lfp_sign_ctrl = (wfsel_tri == 1'b1) ? wf_tri_sign : wf_saw_sign; //AOI

//note that LFA is 8-bit unsigned, LFP is 8-bit sign and magnitude output
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        if(lfa_reg_ld) lfa_reg <= multiplier_sr[15:8];
        if(lfp_reg_ld) lfp_reg <= (pmd_zero == 1'b1) ? 8'h00 : {(multiplier_sr[15] ^ ~lfp_sign_ctrl), multiplier_sr[14:8]};
    end
end

//lfp debug(2's complement)
reg     [7:0]   lfp_reg_debug;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        if(lfp_reg_ld) lfp_reg_debug <= (pmd_zero == 1'b1) ? 8'h00 : 
                                        ((multiplier_sr[15] ^ ~lfp_sign_ctrl) == 1'b1) ? (~multiplier_sr[14:8] + 7'h1) : multiplier_sr[14:8];
    end
end


endmodule