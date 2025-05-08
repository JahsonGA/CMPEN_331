`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/14/2024 05:00:19 PM
// Design Name: 
// Module Name: HW3_testbench
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


module testbench(

    );
    reg clk;
    
    datapath dp(.clk(clk));
    
    initial begin
        clk = 0;
        forever begin
            #10
            clk = ~clk;
        end
    end
endmodule
