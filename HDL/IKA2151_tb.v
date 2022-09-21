`timescale 10ps/10ps
module IKA2151_tb;

reg             EMUCLK = 1'b1;
reg             IC_n = 1'b1;

always #1 EMUCLK = ~EMUCLK;

initial begin
    #30 IC_n <= 1'b0;
    #200 IC_n <= 1'b1;
end

reg     [1:0]   clkdiv = 2'b00;
reg             phiMref = 1'b0;
wire            phiM_PCEN_n = ~(clkdiv == 2'b11);
always @(posedge EMUCLK) begin
    if(clkdiv == 2'b11) begin clkdiv <= 2'b00; phiMref <= 1'b1; end
    else clkdiv <= clkdiv + 2'b01;

    if(clkdiv == 2'b01) phiMref <= 1'b0;
end

IKA2151 main (
    .i_EMUCLK                   (EMUCLK                     ),
    .i_phiM_PCEN_n              (phiM_PCEN_n                ),

    .i_IC_n                     (IC_n                       ),

    .o_phi1                     (                           ),

    .o_SH1                      (                           ),
    .o_SH2                      (                           )
);





endmodule