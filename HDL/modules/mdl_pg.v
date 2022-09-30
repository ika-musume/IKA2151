module mdl_pg
(
    //master clock
    input   wire            i_EMUCLK, //emulator master clock

    //core internal reset
    input   wire            i_MRST_n,

    //internal clock
    input   wire            i_phi1_PCEN_n, //positive edge clock enable for emulation
    input   wire            i_phi1_NCEN_n, //engative edge clock enable for emulation

    //register data
    input   wire    [6:0]   i_KC, //Key Code
    input   wire    [5:0]   i_KF, //Key Fraction
    input   wire    [2:0]   i_PMS, //Pulse Modulation Sensitivity
    input   wire    [1:0]   i_DT2, //Detune 2

    //Vibrato
    input   wire    [7:0]   i_LFP
);


///////////////////////////////////////////////////////////
//////  Clock and reset
////

wire            phi1pcen_n = i_phi1_PCEN_n;
wire            phi1ncen_n = i_phi1_NCEN_n;
wire            mrst_n = i_MRST_n;



///////////////////////////////////////////////////////////
//////  PMS level latch
////

//PMS level latch
reg     [2:0]   pms_level;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        pms_level <= i_PMS;
    end
end



///////////////////////////////////////////////////////////
//////  Extended LFP
////

//adder for tuning
wire            lfp_data_a = (i_PMS == 3'd7) ? ((i_LFP[6] & i_LFP[5]) | (i_LFP[5] & i_LFP[4])) : 
                             (i_PMS == 3'd6) ? (i_LFP[6] & i_LFP[5]) : 1'b0;
wire    [2:0]   lfp_data_b = (i_PMS == 3'd7) ? i_LFP[6:4] : {1'b0, i_LFP[6:5]};
wire            lfp_data_c = (i_PMS == 3'd7) ? i_LFP[6]   : 1'b0;
wire    [3:0]   lfp_adder = lfp_data_a + lfp_data_b + lfp_data_c;

//extended lfp
reg     [7:0]   lfp_ex;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        if(i_PMS == 3'd7) lfp_ex <= {lfp_adder, i_LFP[3:0]};
        else              lfp_ex <= {lfp_adder[2:0], i_LFP[4:0]};
    end
end



///////////////////////////////////////////////////////////
//////  LFP deviance
////

//LFP sign bit for 2's complement conversion
reg             lfp_sign;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        //lfp_sign becomes 1 when PMS > 0 and LFP sign is negative to convert lfp_ex to 2's complement
        lfp_sign <= ~((i_PMS == 3'd0) | ~i_LFP[7]); 
    end
end

reg     [12:0]  lfp_deviance;
wire    [13:0]  lfp_deviance_debug = (lfp_sign == 1'b1) ? (~lfp_deviance + 7'h1) : lfp_deviance;
always @(*) begin
    case(pms_level)
        3'd0: lfp_deviance <= 13'b0;
        3'd1: lfp_deviance <= {11'b0, lfp_ex[6:5]      };
        3'd2: lfp_deviance <= {10'b0, lfp_ex[6:4]      };
        3'd3: lfp_deviance <= {9'b0,  lfp_ex[6:3]      };
        3'd4: lfp_deviance <= {8'b0,  lfp_ex[6:2]      };
        3'd5: lfp_deviance <= {7'b0,  lfp_ex[6:1]      };
        3'd6: lfp_deviance <= {4'b0,  lfp_ex[7:0], 1'b0};
        3'd7: lfp_deviance <= {3'b0,  lfp_ex[7:0], 2'b0};
    endcase
end





///////////////////////////////////////////////////////////
//////  Conversion pipeline
////

//
//  STEP 1: apply LFP value
//

//int/frac adder sets
wire    [6:0]   step1_frac_adder      = i_KF + (lfp_deviance[5:0] ^ {6{lfp_sign}}) + lfp_sign; 
wire    [7:0]   step1_int_adder       = i_KC + (lfp_deviance[12:6] ^ {7{lfp_sign}}) + step1_frac_adder[6];
wire    [2:0]   step1_notegroup_adder = i_KC[1:0] + (lfp_deviance[7:6] ^ {2{lfp_sign}}) + step1_frac_adder[6];
wire    [12:0]  step1_debug_pitchval = (lfp_sign == 1'b0) ? {i_KC, i_KF} + lfp_deviance : {i_KC, i_KF} + ~lfp_deviance + 13'd1;

//final note value
reg     [12:0]  step1_pitchval; //add or subtract LFP value from KC, KF

//flags
reg             step1_pitchval_ovfl;
reg             step1_notegroup_noaddend; //this flag set when no addend is given to a "note group" range(note group: 012/456/89A/CDE)
reg             step1_notegroup_ovfl; //note group overflow, 6(3'b1_10) + 2(3'b0_10)
reg             step1_lfp_sign;

always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        step1_pitchval <= {step1_int_adder[6:0], step1_frac_adder[5:0]};
        step1_pitchval_ovfl <= step1_int_adder[7];

        step1_notegroup_noaddend <= ~(lfp_deviance[6] | lfp_deviance[7]);
        step1_notegroup_ovfl <= step1_notegroup_adder[2];

        step1_lfp_sign <= lfp_sign;
    end
end


//
//  STEP 2: fine tuning
//

//lfp_sign == 0, notegroup ovfl == 1: +1
//lfp_sign == 0, notegroup_ovfl == 0, step1_pitchval[7:6] == 3: +1
//lfp_sign == 0, notegroup_ovfl == 0, step1_pitchval[7:6] != 3: +0
//lfp_sign == 1: +0
wire            step2_int_adder_add1 = ((step1_pitchval[7:6] == 2'd3) | step1_notegroup_ovfl) & ~step1_lfp_sign;

//lfp_sign == 0: -0
//lfp_sign == 1, notegroup_noaddend == 0, notegroup_ovfl == 0: -1
wire            step2_int_adder_sub1 = ~(step1_notegroup_ovfl | step1_notegroup_noaddend | ~step1_lfp_sign);

wire    [7:0]   step2_int_adder = step1_pitchval[12:6] + {7{step2_int_adder_sub1}} + step2_int_adder_add1;

//pitchval
reg     [12:0]  step2_tuned_pitchval;

//flags
reg             step2_tuned_pitchval_ovfl;
reg             step2_pitchval_ovfl;
reg             step2_int_sub1;
reg             step2_lfp_sign;

always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        step2_tuned_pitchval <= {step2_int_adder[6:0], step1_pitchval[5:0]};
        step2_tuned_pitchval_ovfl <= step2_int_adder[7];

        step2_int_sub1 <= step2_int_adder_sub1;

        step2_pitchval_ovfl <= step1_pitchval_ovfl;
        step2_lfp_sign <= step1_lfp_sign;
    end
end


//
//  STEP 3: overflow control
//

reg     [12:0]  step3_clipped_pitchval;

always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        case({step2_lfp_sign, step2_pitchval_ovfl, step2_tuned_pitchval_ovfl, step2_int_sub1})
            //lfp = positive
            4'b0000: step3_clipped_pitchval <= step2_tuned_pitchval;
            4'b0001: step3_clipped_pitchval <= 13'b000_0000_000000; //will not happen
            4'b0010: step3_clipped_pitchval <= 13'b111_1110_111111; //max
            4'b0011: step3_clipped_pitchval <= 13'b111_1110_111111; //will not happen
            4'b0100: step3_clipped_pitchval <= 13'b111_1110_111111;
            4'b0101: step3_clipped_pitchval <= 13'b111_1110_111111; //will not happen
            4'b0110: step3_clipped_pitchval <= 13'b111_1110_111111;
            4'b0111: step3_clipped_pitchval <= 13'b111_1110_111111; //will not happen

            //lfp=negative
            4'b1000: step3_clipped_pitchval <= 13'b000_0000_000000; //min
            4'b1001: step3_clipped_pitchval <= 13'b000_0000_000000;
            4'b1010: step3_clipped_pitchval <= 13'b000_0000_000000;
            4'b1011: step3_clipped_pitchval <= 13'b000_0000_000000;
            4'b1100: step3_clipped_pitchval <= step2_tuned_pitchval;
            4'b1101: step3_clipped_pitchval <= 13'b000_0000_000000;
            4'b1110: step3_clipped_pitchval <= step2_tuned_pitchval;
            4'b1111: step3_clipped_pitchval <= step2_tuned_pitchval;
        endcase
    end
end


//
//  STEP 4: fractional part detuning
//

//dt2 value latch
reg     [1:0]   step4_detune2;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        step4_detune2 <= i_DT2;
    end
end

//fractional part detuning
wire            step4_detune2_frac_add32 = (step4_detune2 == 2'd2) | (step4_detune2 == 2'd3);
wire            step4_detune2_frac_add20 = (step4_detune2 == 2'd3);
wire    [6:0]   step4_frac_adder = step3_clipped_pitchval[5:0] +
                                   {step4_detune2_frac_add32, step4_detune2_frac_add20, 1'b0, step4_detune2_frac_add20, 2'b00};

wire    [5:0]   step4_debug_frac_adder = step4_frac_adder[5:0];
wire    [5:0]   step4_debug_orig = step3_clipped_pitchval[5:0];

//integer part detuning flag
reg             step4_detune2_int_add8;
reg             step4_detune2_int_add4;
reg             step4_detune2_int_add2;
reg             step4_detune2_int_add1;
reg             step4_detune2_int_addc;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        step4_detune2_int_add8 <= ~(step4_detune2 == 2'd0);

        step4_detune2_int_add4 <= (step4_detune2 == 2'd3);

        step4_detune2_int_add2 <= (step4_frac_adder[6] & step3_clipped_pitchval[6] & (step4_detune2 == 2'd2)) |
                                  (step4_frac_adder[6] & step3_clipped_pitchval[7]);

        step4_detune2_int_add1 <= (step4_detune2 == 2'd2);

        step4_detune2_int_addc <= (~step4_frac_adder[6] & step3_clipped_pitchval[7] & (step4_detune2 == 2'd2)) |
                                  (step4_frac_adder[6] & ~((step4_frac_adder[6] & step3_clipped_pitchval[6] & (step4_detune2 == 2'd2)) | //same as add2
                                                           (step4_frac_adder[6] & step3_clipped_pitchval[7])));
    end
end

reg     [12:0]  step4_frac_detuned_pitchval;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        step4_frac_detuned_pitchval <= {step3_clipped_pitchval[12:6], step4_frac_adder[5:0]};
    end
end


//
//  STEP 5: integer part detuning
//

wire    [7:0]   step5_int_adder = step4_frac_detuned_pitchval[12:6] +
                                  {3'b000, step4_detune2_int_add8, step4_detune2_int_add4, step4_detune2_int_add2, step4_detune2_int_add1} +
                                  step4_detune2_int_addc;

reg             step5_int_detuned_pitchval_ovfl;
reg     [12:0]  step5_int_detuned_pitchval;
always @(posedge i_EMUCLK) begin
    if(!phi1ncen_n) begin
        step5_int_detuned_pitchval <= {step5_int_adder[6:0], step4_frac_detuned_pitchval[5:0]};
        step5_int_detuned_pitchval_ovfl <= step5_int_adder[7];
    end
end


//
//  STEP 6: final pitch value
//

wire    [12:0]  step6_final_pitchval = (step5_int_detuned_pitchval_ovfl == 1'b1) ? 13'b111_1110_111111 : step5_int_detuned_pitchval;



endmodule