`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/08/2024 07:52:23 PM
// Design Name: 
// Module Name: Immed_Gen
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module Immed_Gen(
    input logic [31:0] IR,
    output logic [31:0] U_TYPE,
    output logic [31:0] I_TYPE,
    output logic [31:0] S_TYPE,
    output logic [31:0] B_TYPE,
    output logic [31:0] J_TYPE
    );
    
// from our OTTER:
    assign I_TYPE = {{21{IR[31]}}, IR[30:25], IR[24:20]};
    assign S_TYPE = {{21{IR[31]}}, IR[30:25], IR[11:7]};
    assign B_TYPE = {{20{IR[31]}}, IR[7], IR[30:25], IR[11:8], 1'b0};
    assign U_TYPE = { IR[31:12], {12{1'b0}}};
    assign J_TYPE = {{12{IR[31]}}, IR[19:12], IR[20], IR[30:21], 1'b0};  
    
    // assigning immeditate values
// these are DIEGOS values for each output
//    assign I_TYPE = {{21{IR[24]}}, IR[23:18], IR[17:13]};
//    assign S_TYPE = {{21{IR[24]}}, IR[23:18], IR[4:0]};
//    assign B_TYPE = {{20{IR[24]}}, IR[0], IR[23:18], IR[4:1], 1'b0};
//    assign U_TYPE = {IR[24:5], 12'b0};
//    assign J_TYPE = {{12{IR[24]}}, IR[12:5], IR[13], IR[13], IR[23:14], 1'b0};
    
endmodule
