/** @module : writeback
 *  @author : Adaptive & Secure Computing Systems (ASCS) Laboratory
 
 *  Copyright (c) 2018 BRISC-V (ASCS/ECE/BU)
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.

 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *  THE SOFTWARE.
 */
 module writeback_unit (
      clock, reset,  
      opWrite,
      opSel, 
      opReg, 
      ALU_Result, 
      memory_data, 
      write, write_reg, write_data, 
      report
); 
 
parameter  CORE = 0, DATA_WIDTH = 32;
input  clock; 
input  reset; 

input  opWrite; 
input  opSel; 
input  [4:0]  opReg;
input  [DATA_WIDTH-1:0] ALU_Result;
input  [DATA_WIDTH-1:0] memory_data; 

output  write;
output  [4:0]  write_reg;
output  [DATA_WIDTH-1:0] write_data;

input report; 

assign write_data = opSel? memory_data : ALU_Result; 
assign write_reg  = opReg; 
assign write      = opWrite; 

reg [31: 0] cycles; 
always @ (posedge clock) begin 
    cycles <= reset? 0 : cycles + 1; 
    if (report)begin
        $display ("------ Core %d Writeback Unit - Current Cycle %d ----", CORE, cycles); 
        $display ("| opSel       [%b]", opSel);
        $display ("| opReg       [%b]", opReg);
        $display ("| ALU_Result  [%d]", ALU_Result);
        $display ("| Memory_data [%d]", memory_data);
        $display ("| write       [%b]", write);
        $display ("| write_reg   [%d]", write_reg);
        $display ("| write_data  [%d]", write_data);
        $display ("----------------------------------------------------------------------");
    end
end

endmodule

 //------------------------------------------------mem_write test bench-------------------------------------//
module tb_writeback_unit (); 

reg clk, reset;  
reg  opWrite; 
reg  opSel; 
reg  [4:0]  opReg;
reg  [31:0] ALU_Result;
reg  [31:0] memory_data; 

wire  write;
wire  [4:0]  write_reg;
wire  [31:0] write_data;

reg  report; 

writeback_unit #(0, 32) WB (
     .clock(clk),
     .reset(reset), 
     .opWrite(opWrite),
      .opSel(opSel), 
      .opReg(opReg), 
      .ALU_Result(ALU_Result), 
      .memory_data(memory_data), 
      .write(write),
       .write_reg(write_reg),
       .write_data(write_data), 
      .report(report)
);

// Clock generator
always #1 clk = ~clk;

initial begin
  $dumpfile ("writeback.vcd");
  $dumpvars();
  clk   = 0;
  reset = 1;
  opWrite       = 0; 
  opSel         = 0; 
  opReg         = 0; 
  ALU_Result    = 0; 
  memory_data   = 0; 
  report        = 1; 
  
  #10 reset = 0; 
  $display (" --- Start --- ");
  repeat (1) @ (posedge clk);
  
  opWrite       = 1; 
  opSel         = 0; 
  opReg         = 3; 
  ALU_Result    = 5; 
  memory_data   = 9; 
  repeat (1) @ (posedge clk);
  
  opWrite       = 0; 
  opSel         = 1; 
  opReg         = 0; 
  ALU_Result    = 4; 
  memory_data   = 8;
  repeat (1) @ (posedge clk);
  #300 $finish;
  
  end

endmodule
