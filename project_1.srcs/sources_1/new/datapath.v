`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Jahson Gonzalez-Allie
// 
// Create Date: 08/14/2024 05:07:58 PM
// Design Name: 
// Module Name: datapath
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
module datapath(
    input clk
    );
    //=======F STAGE=========
    // pc and pcNext counter
    wire [31:0] pc, nextPc;
    wire [31:0] inst;
    wire [31:0] inst_d;
    wire stall;                                 //holds decsion to stall or not
    
    // pc and pcNext counter
    program_counter pcmodule(.clk(clk), .stall(stall), .nextPc(nextPc), .pc(pc));
    pc_adder pa(.pc(pc), .offset(32'd4), .nextPc(nextPc));
    
    // instruct memory
    inst_mem im(.pc(pc), .inst(inst));
    
    // ====ORDER of functions====
    // Instruct mem
    // ID/ID pipeline
    // Hazard
    // Control and sign exten
    // reg file
    // ID/EX pipeline
    // forward unit
    // 3x1 mux
    // regDstMux 5bit 2x1
    // ALU
    // EX/MEM pipeline
    // Data mem
    // MEM/WB pipeline
    // 2x1 mux write back
    
    
    
    //PIPELINE
    // IF/ID
    // outputs of IF/ID pipeline
  
    f_d_pipeline IF_ID_pipeline(
        .clk(clk),
        .stall(stall),
        .inst(inst),
        
        .inst_d(inst_d)
    );

    //=======D STAGE=========
    // control unit
    wire [5:0] op,func;                                             //holds func and op addrs 6 bits each
    wire aluSrc, memToReg, memWrite, regDst,memRead;                //holds single bit flags 1 bit each
    wire regWrite;                                                  //holds flag 1 bit   
    wire [3:0] aluControl;                                          //holds 4 bit alu value
    
    assign op = inst_d[31:26];    //gets the first 6 bits
    assign func = inst_d[5:0];    //gets 6 bits of func
    
    control_unit ctrl_unit(
        .op(op),
        .func(func),
        .stall(stall),
        
        .regWrite(regWrite),
        .memToReg(memToReg),
        .memWrite(memWrite),
        .aluSrc(aluSrc),
        .regDst(regDst),
        .memRead(memRead),
        .aluControl(aluControl)
        );
        
    // Mux 2:1 5 bits
    // get opcode and func from instruct
    wire [31:0] writeData;                                          //holds addrs to write to 32 bits
    wire [4:0] rs, rt, rd;                                    //holds the reg src, target, dst 5 bits each 
    wire [4:0] writeAddr;                                           //holds addrs for write back

    // Reg file 
    assign rs = inst_d[25:21];    //gets 5 bits of src
    assign rt = inst_d[20:16];    //gets 5 bits of target
    assign rd = inst_d[15:11];    //gets 5 bits of dst
    
    mux_2x1_5b mux2_1_5b(
        .in0(rt_x),
        .in1(rd_x),
        .sel(regDst_x),
        
        .out(writeAddr)            
    );
        
    // Both Requre input from control unit so they need be called called after the unit
    // Use rt for I-type, rd for R-type
    //assign writeAddr = (op == 6'b100011) ? rt : rd;   
    
    //forward muxes here
    //  3x1 MUX 32-bit
    // 3x1 32-bit mux output
    //wire [31:0] out;
    wire [31:0] aluIn1, aluIn2;
   
    mux_3x1_32b mux3x1_A(
        .in0(regOut1_x),    // reg file output
        .in1(writeData),    // forward value for MEM/Wb
        .in2(aluOut_m),       // forward value from EX/MEM
        .sel(forwardA),     // forward control signal
        
        .out(aluIn1)
    );
    
    mux_3x1_32b mux3x1_B(
        .in0(regOut2_x),    // reg file output
        .in1(writeData),    // forward value for MEM/Wb
        .in2(aluOut_m),     // forward value from EX/MEM
        .sel(forwardB),     // forward control signal
        
        .out(aluIn2)
    );

    // Reg file
    wire [31:0] regOut1, regOut2;                                   //holds addrs of reg output 1 and 2 32 bits each
    // NOTE use .<name>(function name) to avoid mistakes
    register_file reg_file(
        .clk(clk), 
        .readAddr1(rs), 
        .readAddr2(rt), 
        .writeAddr(writeAddr_b), 
        .regWrite(regWrite_b), 
        .writeData(writeData), 
        
        .regOut1(regOut1), 
        .regOut2(regOut2)
        );
        
// Sign Extension
    wire [15:0] imm;                                                // Immediate 16-bit
    wire [31:0] imm32;                                              // Immediate 32-bit
    
    assign imm = inst_d[15:0];                                      // gets the immdiate in the last 16-bits
        
    imm_extend ImmExtend(
        .imm(imm),
        
        .imm32(imm32)
        );
        
    //Hazard detection
    // output of hazard 
    hazard_unit hazUnit(
        .rt_x(rt_x),
        .rt_d(rt),
        .rs_d(rs),
        .memRead_x(memRead_x),
        .stall(stall)
    );
            
    //  ID/EX
    // oupts of ID/EX pipeline
    wire [31:0] regOut1_x,regOut2_x, imm32_x;
    wire [4:0] rt_x, rd_x, rs_x;
    wire regWrite_x, memToReg_x, memWrite_x, aluSrc_x, regDst_x, memRead_x;
    wire [3:0] aluControl_x;
    
    d_e_pipeline ID_EX_pipeline(
        .clk(clk),
        .regOut1(regOut1),
        .regOut2(regOut2),
        .imm32(imm32),
        .rt(rt),
        .rd(rd),
        .rs(rs),
        .regWrite(regWrite),
        .memToReg(memToReg),
        .memWrite(memWrite),
        .aluSrc(aluSrc),
        .regDst(regDst),
        .memRead(memRead),
        .aluControl(aluControl),
        
        .regOut1_x(regOut1_x),
        .regOut2_x(regOut2_x),
        .imm32_x(imm32_x),
        .rt_x(rt_x),
        .rd_x(rd_x),
        .rs_x(rs_x),
        .regWrite_x(regWrite_x),
        .memToReg_x(memToReg_x),
        .memWrite_x(memWrite_x),
        .aluSrc_x(aluSrc_x),
        .regDst_x(regDst_x),
        .memRead_x(memRead_x),
        .aluControl_x(aluControl_x)
    );
    
    //=======E STAGE=========
      
    // ======This could be in the wrong place
    //  Forwarding Unit
    // forwarding output
    wire [1:0] forwardA, forwardB;

    forwarding_unit forwardUnit(
        .writeAddr_b(writeAddr_b),
        .writeAddr_m(writeAddr_m),
        .rs_x(rs_x),
        .rt_x(rt_x),
        .regWrite_b(regWrite_b),
        .regWrite_m(regWrite_m),
        .forwardA(forwardA),
        .forwardB(forwardB)
    );
    
    // Mux 2:1 5 bits
    //wire [4:0] in0_5, in1_5, out_5;                                       // 5-bit inputs and ouput
    //wire sel;                                                       // 1-bit selector
    
    // assigns inputs to mux
    //assign in0_5 = rt;
    //assign in1_5 = rd;
    
    // determines what value to pass into selector
    //assign sel = aluSrc;
    
    // Mux 2:1 32 bits
    wire [31:0] in0_32, in1_32, out_32;                                     // 32-bit inputs and ouput
    wire sel;                                                       // 1-bit selector
    
    // assigns inputs to mux
    assign in0_32 = aluIn2;                                            //second output from regfile
    assign in1_32 = imm32_x;                                              //32bit immediate
    assign sel = aluSrc_x;
    
    mux_2x1_32b mux2_1_32b(
        .in0(in0_32),       // I type
        .in1(in1_32),       // r type
        .sel(sel),          // control signal from ID/EX
        
        .out(out_32)        // output to EX/MEM
    );
    
    // ALU
    //aluControl was calculated earlier
    // ALU
    wire [31:0] aluOut;                                // 32-bit inputs and output
    //wire [3:0] aluControl;                                          // 4-bit control for operation
    
    // inputs to ALU are the ourputs of the regester and mux modules
    //assign aluIn1 = regOut1_x; //replace wtih result of forwarding mux
    // assign aluIn2 = out_32;

    alu ALU(
        .aluIn1(aluIn1),
        .aluIn2(out_32),
        .aluControl(aluControl_x),
        
        .aluOut(aluOut)
    );
    
    //  EX/Mem
    // outputs of EX/Mem pipeline
    wire [31:0] aluOut_m, regOut2_m;
    wire [4:0] writeAddr_m;
    wire regWrite_m, memToReg_m, memWrite_m, memRead_m;

    e_m_pipeline EX_MEM_pipeline (
        .clk(clk),
        .aluOut(aluOut),
        .regOut2_x(regOut2_x),
        .writeAddr(writeAddr),
        .memToReg_x(memToReg_x),
        .regWrite_x(regWrite_x),
        .memWrite_x(memWrite_x),
        .memRead_x(memRead_x),
        
        .aluOut_m(aluOut_m),
        .regOut2_m(regOut2_m),
        .writeAddr_m(writeAddr_m),
        .regWrite_m(regWrite_m),
        .memToReg_m(memToReg_m),
        .memWrite_m(memWrite_m),
        .memRead_m(memRead_m)    
    );
    
    //=======M STAGE=========
    
    
    // determines what value to pass into selector
    //assign sel = aluSrc;

    // redundant with the mux3X1_B
    /*mux_2x1_32b mux2_1_32b(
        .in0(in0_32),
        .in1(in1_32),
        .out(out_32),
        .sel(sel)    
    );*/
       
    
    //Data Memory
    //wire clk, memWrite, memRead;                                  // 1-bit clk, memWrite, memRead signals
    wire [31:0] addr, memIn, memOut;                                // 32-bit address and memory input and output locations

    assign addr = aluOut_m;                                           //set address for DM
    assign memIn = regOut2_m;                                         //input is from the regester
    
    data_mem DataMem(
        .clk(clk),
        .memWrite(memWrite_m),
        .memRead(memRead_m),
        
        .addr(addr),
        .memIn(memIn),
        .memOut(memOut)
    );

//  Mem/WB   
// output of Mem/WB
    wire [31:0] aluOut_b, memOut_b;
    wire [4:0] writeAddr_b;
    wire regWrite_b, memToReg_b;
     
    m_w_pipeline MEM_WB_pipeline(
        .clk(clk),
        .aluOut_m(aluOut_m),
        .memOut(memOut),
        .writeAddr_m(writeAddr_m),
        .regWrite_m(regWrite_m),
        .memToReg_m(memToReg_m),
        .aluOut_b(aluOut_b),
        .memOut_b(memOut_b),
        .writeAddr_b(writeAddr_b),
        .regWrite_b(regWrite_b),
        .memToReg_b(memToReg_b)    
    );
    
    //=======W STAGE=========
    
    // determines if the data is from the memory or ALU
    //assign writeData = memToReg ? memOut : aluOut;
    
    //===HW 5===
    // MUX for memToReg
    mux_2x1_32b memToRegMux(
        .in0(aluOut_b),                                             // ALU output
        .in1(memOut_b),                                             // Memory output
        .sel(memToReg_b),                                           // Control signal from MEM/WB
        
        .out(writeData)                                             // Output to be written to the register file
    );
    
    // determines what value to pass into selector
    //assign sel = aluSrc;
  
endmodule

/* ================= Modules to implement for HW1 =====================*/
module program_counter(
    input clk, stall,
    input [31:0] nextPc,
    output reg [31:0] pc
    );
    initial begin
        pc = 32'd96; // PC initialized to start from 100.
    end
    // ==================== Students fill here BEGIN ====================
    //update pc on positive edge of the clock
    always @(posedge clk) begin
        if (!stall) begin
            pc <= nextPc;
        end
    end
    // ==================== Students fill here END ======================
endmodule

module pc_adder(
    input [31:0] pc, offset,
    output reg [31:0] nextPc
    );
    // ==================== Students fill here BEGIN ====================
    //add pc and offset to comput the nextPc
    always @(*) begin
        nextPc = pc + offset;
    end
    // ==================== Students fill here END ======================
endmodule

/* ================= Modules to implement for HW3 =====================*/
module inst_mem(
    input [31:0] pc,
    output reg [31:0] inst
    );
    
    // This is an instruction memory that holds 64 instructions, 32b each.
    reg [31:0] memory [0:63];
    
    // Initializing instruction memory.
    initial begin       
        //memory[25] = {6'b100011, 5'd0, 5'd1, 16'd0};                      // lw($1, 0($0))
        //memory[26] = {6'b100011, 5'd0, 5'd2, 16'd4};                      // lw $1, 4($0) 
        //memory[27] = {6'b000000, 5'd1, 5'd2, 5'd3, 11'b00000100010};      // sub $3, $1, $2 
        //memory[28] = {6'b100011, 5'd3, 5'd4, 16'hFFFC};                   // lw $4, -4($3) 
        
        memory[25] = {6'b100011, 5'd0, 5'd1, 16'd0};                        // lw($1, 0($0))
        memory[26] = {6'b100011, 5'd0, 5'd2, 16'd4};                        // lw($2, 4($0))
        memory[27] = {6'b100011, 5'd0, 5'd3, 16'd8};                        // lw($3, 8($0))
        memory[28] = {6'b100011, 5'd0, 5'd4, 16'd16};                       // lw($4, 16($0))
        memory[29] = {6'b000000, 5'd1, 5'd2, 5'd5, 11'b00000100000};        // add $5, $1, $2
        memory[30] = {6'b100011, 5'd3, 5'd6, 16'hFFFC};                     // lw($6, -4($3))
        memory[31] = {6'b000000, 5'd4, 5'd3, 5'd7, 11'b00000100010};        // sub $7, $4, $3
        
        //memory[25] = {6'b100011, 5'd0, 5'd1, 16'd0};                        // lw $1, 0($0)
        //memory[26] = {6'b100011, 5'd0, 5'd2, 16'd4};                        // lw $2, 4($0)
        //memory[27] = {6'b100011, 5'd0, 5'd4, 16'd16};                       // lw $4, 16($0)
        //memory[28] = {6'b000000, 5'd1, 5'd2, 5'd3, 11'b00000100010};        // sub $3, $1, $2
        //memory[29] = {6'b100011, 5'd3, 5'd4, 16'hFFFC};                     // lw $4, -4($3)

    end
    // ==================== Students fill here BEGIN ====================

    // no shifting or division
    // since PC is divided by 4 the last two 2 bits aren't needed.
    always @(*) begin
        inst = memory[pc[7:2]];
    end
    // ==================== Students fill here END ======================
endmodule

module register_file(
    input [4:0] readAddr1, readAddr2, writeAddr,
    input [31:0] writeData,
    input regWrite, clk,
    output reg [31:0] regOut1, regOut2
    );
    
    // Initializing registers. Do not touch here.
    reg [31:0] register [0:31]; // 32 registers, 32b each.
    integer i;
    initial begin
        for (i=0; i<32; i=i+1) begin
            register[i] = 32'd0; // Initialize to zero
        end
    end
    // ==================== Students fill here BEGIN ====================
    //reading
    // update values when the address changes
    always @(*) begin
        regOut1 = register[readAddr1];  // gets address of first reg and outputs
        regOut2 = register[readAddr2];  // gets address of second reg and outputs
    end
    //writing
    // update when regWrite changes
    // always @(*) is used for combination logic like arithmetic, multiplexing, or ALU operates without memory
    // always @(posedge clk) is used for sequential logic. which is why it is used with reg
    always @(negedge clk) begin
        if (regWrite /*&& writeData !== 32'bx*/) begin
            register[writeAddr] <= writeData;  // write data to reg with writeAddr
        end
    end
    // ==================== Students fill here END ======================
endmodule

module control_unit(
    input stall,
    input [5:0] op, func,
    output reg regWrite, memToReg, memWrite, aluSrc, regDst, memRead,
    output reg [3:0] aluControl
    );
    // ==================== Students fill here BEGIN ====================
    initial begin
        regWrite = 1'b0;
        memWrite = 1'b0;
        memRead = 1'b0;
    end
    
    always @(*) begin
        if (!stall) begin
            case (op)
                6'b000000: begin // r-type
                    regWrite = 1'b1;
                    memToReg = 1'b0;
                    aluSrc = 1'b0;
                    regDst = 1'b1;
                    memWrite = 1'b0;
                    memRead = 1'b0;
                    case (func)
                        6'b100000: begin // add
                            aluControl = 4'b0010;
                        end
                        6'b100010: begin // sub
                            aluControl = 4'b0110;
                        end
                        default: begin
                            aluControl = 4'bxxxx;
                        end
                    endcase
                end
                6'b100011: begin // lw
                    regWrite = 1'b1;
                    memToReg = 1'b1;
                    aluSrc = 1'b1;
                    regDst = 1'b0;
                    memWrite = 1'b0;
                    memRead = 1'b1;
                    aluControl = 4'b0010;
                end
                6'b101011: begin // sw
                    regWrite = 1'b0;
                    memToReg = 1'bx;
                    aluSrc = 1'b1;
                    regDst = 1'bx;
                    memWrite = 1'b1;
                    memRead = 1'b0;
                    aluControl = 4'b0010;
                end
                default: begin
                    regWrite = 1'b0;
                    memToReg = 1'bx;
                    aluSrc = 1'bx;
                    regDst = 1'bx;
                    memWrite = 1'b0;
                    memRead = 1'b0;
                    aluControl = 4'bxxxx;
                end
            endcase
        end 
        else begin
            regWrite = 1'b0;
            memToReg = 1'b0;
            aluSrc = 1'b0;
            regDst = 1'b0;
            memWrite = 1'b0;
            memRead = 1'b0;
            aluControl   = 4'b0000;
        end
    end

    // ==================== Students fill here END ======================
endmodule

/* ================= Modules to implement for HW4 =====================*/
module imm_extend(
    input [15:0] imm,
    output reg [31:0] imm32
    );
    // ==================== Students fill here BEGIN ====================
    // imm32 is equal to imm but the upper 15 bits is a clone of the MSB of the imm.
    //  this is so the sign is maintained for the bninary number. 0
    
    always @(*) begin
    
        // The imm[15] gets the MSB and the 16 outside the curly brace dupes it 16time. the outer most {}
        //  are used to "rebuild" the immediate
        imm32 = {{16{imm[15]}}, imm};
    
    end

    // ==================== Students fill here END ======================
endmodule

module mux_2x1_32b(
    input [31:0] in0, in1,
    input sel,
    output reg [31:0] out
    );
    // ==================== Students fill here BEGIN ====================
    always @(*) begin
        if(sel) begin
            out = in1;      //output when selector bit is 0
        end else begin
            out = in0;      //output when selector bit is 1
        end
    end
    // ==================== Students fill here END ======================
endmodule

module alu(
    input [31:0] aluIn1, aluIn2,
    input [3:0] aluControl,
    output reg [31:0] aluOut
    );

    // ==================== Students fill here BEGIN ====================
    // addition 0010
    // subtraction 0110
    // aluControl controls the operation being done within ALU 
    always @(*) begin
        case(aluControl)
            4'b0000: aluOut = aluIn1 & aluIn2;
            4'b0001: aluOut = aluIn1 | aluIn2;
            4'b0010: aluOut = aluIn1 + aluIn2;
            4'b0110: aluOut = aluIn1 - aluIn2;
            4'b0111: aluOut = aluIn1 < aluIn2;
            4'b1100: aluOut = ~(aluIn1 | aluIn2);
            default: aluOut = 32'hxxxx_xxxx;
        endcase
    end

    // ==================== Students fill here END ======================
endmodule

module data_mem(
    input clk, memWrite, memRead,
    input [31:0] addr, memIn,
    output reg [31:0] memOut
    );
    
    reg [31:0] memory [0:63]; // 64x32 memory
    
    // Initialize data memory. Do not touch this part.
    initial begin
        memory[0] = 32'd16817;
        memory[1] = 32'd16801;
        memory[2] = 32'd16;
        memory[3] = 32'hDEAD_BEEF;
        memory[4] = 32'h4242_4242;
    end
    
    // ==================== Students fill here BEGIN ====================
    // Read Memory
    always @(*) begin
        // if memRead is ture and the adrress accessed is less than the size of the memory, 64 words
        if(memRead) begin
            memOut <= memory[addr[31:2]];
        end 
        else begin
            memOut = 32'hxxxx_xxxx;  // X when not reading
        end
    end
    // Memory write should be done the falling edge
    always @(negedge clk) begin
        // if memRead is ture and the adrress accessed is less than the size of the memory, 64 
        if(memWrite) begin
            // write to memory and ensure it word aligned 32-bit
            // a word is 32-bits but the last two bits are offset since the last 2 bits in the words 
            //  cannot be used to represent a memory index.  
            
            // non-blocking assignment is used when using the clock edge
            //  it ensures that assignment take effect at the end of clock
            // Extract the word address from the 32-bit address          
            memory[addr[31:2]] <= memIn;
        end
    end
    
    // ==================== Students fill here END ======================
endmodule

/* ================= Modules to implement for HW5 =====================*/
module mux_2x1_5b(
    input [4:0] in0, in1,
    input sel,
    output reg [4:0] out
    );
    // ==================== Students fill here BEGIN ====================
    always @(*) begin
        if (sel)
            out <= in1;
        else
            out <= in0;
    end
    // ==================== Students fill here END ======================
endmodule

//Final Project
module f_d_pipeline (
    input clk,
    input stall,
    input [31:0] inst,
    output reg [31:0] inst_d
);
    
//    initial begin
//        inst_d = 32'd0;
//    end 
    //update on pos clk to sync
    always @(posedge clk) begin
        if (!stall) begin
            // if no stalling needed assign the instruction to the pipeline
            inst_d <= inst;
        end
    end
endmodule

module d_e_pipeline (
    input clk,
    input [31:0] regOut1, regOut2, imm32,
    input [4:0] rt, rd, rs,
    input regWrite, memToReg, memWrite, aluSrc, regDst, memRead,
    input [3:0] aluControl,
    output reg [31:0] regOut1_x, regOut2_x, imm32_x,
    output reg [4:0] rt_x , rd_x, rs_x,
    output reg regWrite_x , memToReg_x, memWrite_x, aluSrc_x, regDst_x, memRead_x,
    output reg [3:0] aluControl_x
);

    initial begin
        {regWrite_x, memWrite_x, memRead_x} = 3'd0;
    end
    //update on pos clk to sync
    always @(posedge clk) begin
        //send values into pipeline
        regOut1_x <= regOut1;
        regOut2_x <= regOut2;
        imm32_x <= imm32;
        rt_x <= rt;
        rd_x <= rd;
        rs_x <= rs;
        regWrite_x <= regWrite;
        memToReg_x <= memToReg;
        memWrite_x <= memWrite;
        aluSrc_x <= aluSrc;
        regDst_x <= regDst;
        memRead_x <= memRead;
        aluControl_x <= aluControl;
    end
endmodule

module e_m_pipeline (
    input clk,
    input [31:0] aluOut, regOut2_x,
    input [4:0] writeAddr,
    input regWrite_x, memToReg_x, memWrite_x, memRead_x,
    output reg [31:0] aluOut_m, regOut2_m,
    output reg [4:0] writeAddr_m,
    output reg regWrite_m, memToReg_m, memWrite_m, memRead_m
);
    
    initial begin
        {regWrite_m, memWrite_m, memRead_m} = 3'd0;
    end
    
    //update on pos clk to sync
    always @(posedge clk) begin
        //send values into pipeline
        aluOut_m <= aluOut;
        regOut2_m <= regOut2_x;
        writeAddr_m <= writeAddr;
        regWrite_m <= regWrite_x;
        memToReg_m <= memToReg_x;
        memWrite_m <= memWrite_x;
        memRead_m <= memRead_x;
    end
endmodule

module m_w_pipeline (
    input clk,
    input [31:0] aluOut_m, memOut,
    input [4:0] writeAddr_m,
    input regWrite_m, memToReg_m,
    output reg [31:0] aluOut_b, memOut_b,
    output reg [4:0] writeAddr_b,
    output reg regWrite_b, memToReg_b
);

    initial begin
        regWrite_b = 1'd0;
    end
    //update on pos clk to sync
    always @(posedge clk) begin
        //send values into pipeline
        aluOut_b <= aluOut_m;
        memOut_b <= memOut;
        writeAddr_b <= writeAddr_m;
        regWrite_b <= regWrite_m;
        memToReg_b <= memToReg_m;
    end
endmodule

module forwarding_unit (
    input [4:0] writeAddr_b, writeAddr_m, rs_x, rt_x,
    input regWrite_b, regWrite_m,
    output reg [1:0] forwardA, forwardB
);
    // when every there is a change update value
    always @(*) begin
        forwardA= 2'b00;
        forwardB= 2'b00;
        // Forwarding logic for source operands
        forwardA = (regWrite_m && (writeAddr_m != 0) && (writeAddr_m == rs_x)) ? 2'b10 :// forward from MEM
                   (regWrite_b && (writeAddr_b != 0) && (writeAddr_b == rs_x)) ? 2'b01 : 2'b00;// from WB or not at all

        forwardB = (regWrite_m && (writeAddr_m != 0) && (writeAddr_m == rt_x)) ? 2'b10 : // forward from MEM
                   (regWrite_b && (writeAddr_b != 0) && (writeAddr_b == rt_x)) ? 2'b01 : 2'b00;// from WB or not at all
    end
endmodule

module mux_3x1_32b (
    input [31:0] in0, in1, in2,
    input [1:0] sel,
    output reg [31:0] out
);
    always @(*) begin
        case (sel)
            2'b00: out = in0;
            2'b01: out = in1;
            2'b10: out = in2;
            default: out = 32'bx;
        endcase
    end
endmodule

module hazard_unit (
    input [4:0] rt_x, rt_d, rs_d,
    input memRead_x,
    output reg stall
);
    always @(*) begin
        // stall = 1 if memRead is 1 and rt = rs or rt = rd of the next income instruction. 
        stall = (memRead_x && ((rt_x == rs_d) || (rt_x == rt_d))) ? 1:0;
           
    end
endmodule