`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: OTTER_CPU
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
typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        
typedef struct packed{
    opcode_t opcode;
    logic [2:0] func3;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc;
} instr_t;

module OTTER_MCU(input CLK,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
    wire [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,A,B,
        I_immed,S_immed,U_immed,aluBin,aluAin,aluResult,rfIn,csr_reg, mem_data;
    wire ERR;
    wire [31:0] IR, wd;
    wire memRead1,memRead2;
    wire pcWrite,memWrite, op1_sel,mem_op,IorD,pcWriteCond,memRead;
    wire [1:0] opB_sel, rf_sel, wb_sel, mSize;
    wire [1:0] pc_sel;
    wire [3:0]alu_fun;
    wire opA_sel;
    wire ZERO;
    
    logic [31:0] execute_Bin;
    logic [31:0] execute_rs2;
    logic [31:0] memory_rs2;
    logic [31:0] execute_aluRes;
    logic [31:0] memory_aluRes;
    logic [31:0] execute_Ain;
    logic [31:0] writeback_aluRes;
    logic [31:0] memory_dout2;
    logic [31:0] writeback_dout2;
    

    //PC is byte-addressed but our memory is word addressed 
    ProgCount PC (.PC_CLK(CLK), .PC_RST(RESET), .PC_LD(pcWrite),
                 .PC_DIN(pc_value), .PC_COUNT(pc)); 
    
    // 4 to 1 mux for PC selct
    Mult4to1 PCdatasrc (next_pc, jalr_pc, branch_pc, jump_pc, pc_sel, pc_value);
    
    // Instantiate instruction memory and data memory. IM uses address and output 1, DM uses addresses and output 2
    //MEMORY STAGE, STUFF SHOUDLD COME FROM MEMORY STRUCT
    OTTER_mem_dualport MEMORY(CLK, pc, memory_aluRes, memory_rs2, memory.memWrite, memRead1, memory.memRead2, ERR, IR, memory_dout2, IOBUS_IN, IOBUS_WR);
 
    // defining inputs to CU Decoder from IR
    logic [6:0] opcode;
    logic func7;
    logic [2:0] func3;
    assign opcode = IR[6:0];
    assign func7 = IR[30];
    assign func3 = IR[14:12];
    // NEW CU DECODER
    CU_DCDR Decoder( opcode, func7, func3, decode.alu_fun, pc_sel, opA_sel, opB_sel, decode.rf_wr_sel, decode.regWrite, decode.memWrite, decode.memRead2);
   
   // defining inputs to IMMED_GEN and creating wires for outputs
   logic [24:0] IRin;
   logic [31:0] U_TYPE;
   logic [31:0] I_TYPE;
   logic [31:0] S_TYPE;
   logic [31:0] B_TYPE;
   logic [31:0] J_TYPE;
   assign IRin = IR[24:0];
   //assign I_TYPE = {{21{IR[24]}}, IR[23:18], IR[17:13]};
   
   // IMMED_GEN Instantiation
   Immed_Gen ImmedGen(IRin, U_TYPE, I_TYPE, S_TYPE, B_TYPE, J_TYPE);
   
    // Creates a 4-to-1 multiplexor used to select the B input of the ALU
    Mult4to1 ALUBinput (B, I_TYPE, S_TYPE, pc, opB_sel, execute_Bin);
    
    Mult2to1 ALUAinput (A, U_TYPE, opA_sel, execute_Ain);
    // Creates a RISC-V ALU
    // Inputs are ALUCtl (the ALU control), ALU value inputs (ALUAin, ALUBin)
    // Outputs are ALUResultOut (the 64-bit output) and Zero (zero detection output)
    OTTER_ALU ALU (execute.alu_fun, execute_Ain, execute_Bin, execute_aluRes, ZERO); // the ALU
    
    // Creates a RISC-V register file
    OTTER_registerFile RF (IR[19:15], IR[24:20], writeback.rd_addr, rfIn, writeback.regWrite, A, B, CLK); // Register file
 
    //Creates 4-to-1 multiplexor used to select reg write back data
    Mult4to1 regWriteback (next_pc,csr_reg, writeback_dout2, writeback_aluRes, writeback.rf_wr_sel, rfIn);
  
    //pc target calculations 
    assign next_pc = pc + 4;    //PC is byte aligned, memory is word aligned
    assign jalr_pc = I_immed + A;
    assign branch_pc = pc + {{21{IR[31]}},IR[7],IR[30:25],IR[11:8] ,1'b0};   //word aligned addresses
    assign branch_pc = pc + {{20{IR[31]}},IR[7],IR[30:25],IR[11:8],1'b0};   //byte aligned addresses
    assign jump_pc = pc + {{12{IR[31]}}, IR[19:12], IR[20],IR[30:21],1'b0};
    assign int_pc = 0;
    
    logic br_lt,br_eq,br_ltu;
              
//==== Instruction Fetch ===========================================
     instr_t decode;
     instr_t execute;
     instr_t memory;
     instr_t writeback;
     
     
     assign pcWrite = 1'b1; 	//Hardwired high, assuming now hazards
     assign memRead1 = 1'b1; 	//Fetch new instruction every cycle
    
     
     
//==== Instruction Decode ===========================================
    
//    logic [31:0] de_ex_opA;
//    logic [31:0] de_ex_opB;
//    logic [31:0] de_ex_rs2;

//    instr_t de_ex_inst, de_inst;
    
//    assign de_inst.rs1_addr=IR[19:15];
//    assign de_inst.rs2_addr=IR[24:20];
//    assign de_inst.rd_addr=IR[11:7];
   
//    assign de_inst.rs1_used=    de_inst.rs1 != 0
//                                && de_inst.opcode != LUI
//                                && de_inst.opcode != AUIPC
//                                && de_inst.opcode != JAL;

     
     assign decode.pc = pc;
     assign decode.opcode = opcode_t'(IR[6:0]);
     assign decode.rs1_addr = IR[19:15];
     assign decode.rs2_addr = IR[24:20];
     assign decode.rd_addr = IR[11:7];
     assign decode.func3 = func3;
//         decode.rs1_used; FOR HAZARDS
//         decode.rs2_used; FOR HAZARDS
//         decode.rd_used; // FOR HAZARDS
//     assign  decode.alu_fun = 0;
//     assign  decode.memWrite = 0;
//     assign  decode.memRead2 = 0;
//     assign  decode.regWrite = 0;
//     assign  decode.rf_wr_sel = 0;
//         decode.mem_type;  //sign, size // JOSEPH IMPLEMENTATION
	
	
//==== Execute ======================================================
     always_ff @(posedge CLK) begin
             execute <= decode;
     end
      
//     logic [31:0] execute_rs2 = 0;
//     logic [31:0] memory_rs2 = 0;
//     logic [31:0] execute_aluRes = 0;
//     logic [31:0] memory_aluRes = 0;
//     logic [31:0] execute_Ain = 0;
//     logic [31:0] execute_Bin;
//     logic [31:0] writeback_aluRes = 0;
     
     //logic [31:0] opA_forwarded;
     //logic [31:0] opB_forwarded;
     





//==== Memory ======================================================
    always_ff @(posedge CLK) begin
             memory <= execute; // happens simultaneously
             memory_aluRes <= execute_aluRes; // happens simultaneously
             memory_rs2 <= execute_rs2; // happens simultaneously
    end
     
     
    assign IOBUS_ADDR = execute_aluRes;
    assign IOBUS_OUT = execute_rs2;
    
//    logic [31:0] memory_dout2 = 0;
//    logic [31:0] writeback_dout2 = 0;
    
 
 
 
     
//==== Write Back ==================================================
     always_ff @(posedge CLK) begin
             writeback <= memory;
             writeback_dout2 <= memory_dout2;
             writeback_aluRes <= memory_aluRes;
     end



       
        
endmodule
