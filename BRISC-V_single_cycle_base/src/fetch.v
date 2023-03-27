/** @module : fetch_unit
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
/** @module : memory_interface
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
 //(* ram_style = "block" *)
module BSRAM (
        clock,
        reset,
        readEnable,
        readAddress,
        readData,
        writeEnable,
        writeAddress,
        writeData, 
        report
); 
parameter CORE = 0, DATA_WIDTH = 32, ADDR_WIDTH = 8;
parameter MEM_DEPTH = 1 << ADDR_WIDTH;

input clock; 
input reset; 
input readEnable;
input [ADDR_WIDTH-1:0]   readAddress;
output [DATA_WIDTH-1:0]  readData;
input writeEnable;
input [ADDR_WIDTH-1:0]   writeAddress;
input [DATA_WIDTH-1:0]   writeData;
input report; 
 
wire [DATA_WIDTH-1:0]     readData;
reg  [DATA_WIDTH-1:0]     sram [0:MEM_DEPTH-1];
 
//--------------Code Starts Here------------------ 
assign readData = (readEnable & writeEnable & (readAddress == writeAddress))? 
                  writeData : readEnable? sram[readAddress] : 0;

always@(posedge clock) begin : RAM_WRITE
    if(writeEnable)
        sram[writeAddress] <= writeData;
end

reg [31: 0] cycles; 
always @ (posedge clock) begin 
    cycles <= reset? 0 : cycles + 1; 
    if (report)begin
        $display ("------ Core %d SBRAM Unit - Current Cycle %d --------", CORE, cycles); 
        $display ("| Read        [%b]", readEnable);
        $display ("| Read Address[%h]", readAddress);
        $display ("| Read Data   [%h]", readData);
        $display ("| Write       [%b]", writeEnable);
        $display ("| Write Addres[%h]", writeAddress);
        $display ("| Write Data  [%h]", writeData);
        $display ("----------------------------------------------------------------------");
    end 
 end    

endmodule


 //-----------------------------------------------------MEM_INTERFACE----------------------------//
module mem_interface (
                     clock, reset,
                     read, write, write_address, read_address, in_data,
                     out_addr, out_data, valid, ready,
                     report
);
parameter CORE = 0, DATA_WIDTH = 32, INDEX_BITS = 6,
                     OFFSET_BITS = 3, ADDRESS_BITS = 20;

input clock, reset;
input read, write;
input [ADDRESS_BITS-1:0] write_address;
input [ADDRESS_BITS-1:0] read_address;
input [DATA_WIDTH-1:0]   in_data;
output valid, ready;
output[ADDRESS_BITS-1:0] out_addr;
output[DATA_WIDTH-1:0]   out_data;

input  report;


BSRAM #(CORE, DATA_WIDTH, ADDRESS_BITS) RAM (
        .clock(clock),
        .reset(reset),
        .readEnable(read),
        .readAddress(read_address),
        .readData(out_data),

        .writeEnable(write),
        .writeAddress(write_address),
        .writeData(in_data),

        .report(report)
);

assign out_addr = read? read_address : 0;
assign valid    = (read | write)? 1 : 0;
assign ready    = (read | write)? 0 : 1; /// Just for testing now

reg [31: 0] cycles;
always @ (posedge clock) begin
    cycles <= reset? 0 : cycles + 1;
    if (report)begin
        $display ("------ Core %d Memory Interface - Current Cycle %d --", CORE, cycles);
        
        $display ("| Read Address     [%h]", read_address);
        $display ("| Read        [%b]", read);
        $display ("| Write Address     [%h]", write_address);
        $display ("| Write       [%b]", write);
        $display ("| Out Data    [%h]", out_data);
        $display ("| In Data     [%h]", in_data);
        $display ("| Ready       [%b]", ready);
        $display ("| Valid       [%b]", valid);
        $display ("----------------------------------------------------------------------");
    end
end

endmodule

//---------------------------------fetch unit------------------------------------------------------//
module fetch_unit(
        clock, reset, start,

        PC_select,
        program_address,
        JAL_target,
        JALR_target,
        branch,
        branch_target,

        // In-System Programmer Interface
        isp_address,
        isp_data,
        isp_write,

        instruction,
        inst_PC,
        valid,
        ready,
        report
);

parameter CORE = 0, DATA_WIDTH = 32, INDEX_BITS = 4,
                     OFFSET_BITS = 3, ADDRESS_BITS = 28;

input clock, reset, start;
input [1:0] PC_select;

input [ADDRESS_BITS-1:0] program_address;
input [ADDRESS_BITS-1:0] JAL_target;
input [ADDRESS_BITS-1:0] JALR_target;
input branch;
input [ADDRESS_BITS-1:0] branch_target;

// In-System Programmer Interface
input [ADDRESS_BITS-1:0] isp_address;
input [DATA_WIDTH-1:0] isp_data;
input isp_write;

output [DATA_WIDTH-1:0]   instruction;
output [ADDRESS_BITS-1:0] inst_PC;
output valid;
output ready;

input report;

reg [ADDRESS_BITS-1:0] old_PC;
reg fetch;

reg  [ADDRESS_BITS-1:0] PC_reg;
wire [ADDRESS_BITS-1:0] PC = reset? program_address : PC_reg;
wire [ADDRESS_BITS-1:0] PC_plus4 = PC + 4;

//Adjustment to be word addressable instruction addresses
wire [ADDRESS_BITS-1:0] inst_addr = PC >> 2;
wire [ADDRESS_BITS-1:0] out_addr;
assign inst_PC =  out_addr << 2;

mem_interface #(CORE, DATA_WIDTH, INDEX_BITS, OFFSET_BITS, ADDRESS_BITS)
                    i_mem_interface (
                     .clock(clock),
                     .reset(reset),
                     .read(fetch),
                     .write(isp_write),
                     .write_address(isp_address),
                     .read_address(inst_addr),
                     .in_data(isp_data),
                     .out_addr(out_addr),
                     .out_data(instruction),
                     .valid(valid),
                     .ready(ready),
                     .report(report)
);

always @ (posedge clock) begin
      if (reset) begin 
        fetch        <= 0; 
        PC_reg       <= 0;  
        old_PC       <= 0; 
      end 
      else begin 
        if (start) begin 
            fetch        <= 1;
            PC_reg       <= program_address;            
            old_PC       <= 0; 
        end 
        else begin 
            fetch        <= 1;
            PC_reg       <= (PC_select == 2'b10)?  JAL_target: 
                            (PC_select == 2'b11)?  JALR_target: 
                            ((PC_select == 2'b01)& branch)?  branch_target : PC_plus4;  
            old_PC       <= PC_reg; 
        end
      end
end

reg [31: 0] cycles; 
always @ (posedge clock) begin 
    cycles <= reset? 0 : cycles + 1; 
    if (report)begin
        $display ("------ Core %d Fetch Unit - Current Cycle %d --------", CORE, cycles); 
        $display ("| Prog_Address[%h]", program_address);
        $display ("| Control     [%b]", PC_select);
        $display ("| PC          [%h]", PC);
        $display ("| old_PC      [%h]", old_PC);
        $display ("| PC_plus4    [%h]", PC_plus4);
        $display ("| JAL_target  [%h]", JAL_target);
        $display ("| JALR_target [%h]", JALR_target);
        $display ("| Branch      [%b]", branch);
        $display ("| branchTarget[%h]", branch_target);
        $display ("| Read        [%b]", fetch);
        $display ("| instruction [%h]", instruction);
        $display ("| inst_PC     [%h]", inst_PC);
        $display ("| Ready       [%b]", ready);
        $display ("| Valid       [%b]", valid);
        $display ("----------------------------------------------------------------------");
    end
end

endmodule
//----------------------------------testbench--------------------------------------------------------//

/** @module : tb_fetch_unit
 *  @author : Adaptive & Secure Computing Systems (ASCS) Laboratory
 *  Copyright (c) 2018 BRISC-V (ASCS/ECE/BU)
 */

module tb_fetch_unit (); 

reg clk, reset, start; 
reg [1:0] PC_select;

reg [7:0] program_address;
reg [7:0] JAL_target;
reg [7:0] JALR_target;
reg branch; 
reg [7:0] branch_target;
reg report;

wire [31:0]   instruction;
wire [7:0]    inst_PC;  
wire valid; 
wire ready; 

//module fetch_unit #(parameter CORE = 0, DATA_WIDTH = 32, INDEX_BITS = 6, 
//                     OFFSET_BITS = 3, ADDRESS_BITS = 20)
                     
fetch_unit IF (
        .clock(clk),
        .reset(reset),
        .start(start),
        
        .PC_select(PC_select),
        .program_address(program_address), 
        .JAL_target(JAL_target),
        .JALR_target(JALR_target),
        .branch(branch),
        .branch_target(branch_target), 
        
        .isp_address(isp_address),
        .isp_data(isp_data),
        .isp_write(isp_write),
        
        .instruction(instruction), 
        .inst_PC(inst_PC),
        .valid(valid),
        .ready(ready),
        .report(report)
); 

// Clock generator
always #1 clk = ~clk;

initial begin
  $dumpfile ("fetch_unit.vcd");
  $dumpvars();
  clk   = 0;
  reset = 1;
  start = 1;
  
  program_address = 0; 
  JAL_target      = 0; 
  JALR_target     = 0; 
  branch          = 0; 
  branch_target   = 2; 
  PC_select       = 0; 
  report          = 1; 
  
  #8  reset = 0; 
  #10 start = 0;
  $display (" --- Start --- ");
  repeat (1) @ (posedge clk);
  
  PC_select<= 1;
  repeat (1) @ (posedge clk);
  
  PC_select<= 2;
  repeat (1) @ (posedge clk);
  
  PC_select<= 3;
  repeat (1) @ (posedge clk);
  
  PC_select<= 4;
  repeat (1) @ (posedge clk);
  
  PC_select<= 5;
  repeat (1) @ (posedge clk);
  
  PC_select<= 1;
  repeat (1) @ (posedge clk);
  #200 $finish;
  end

endmodule
