/** @module : memory
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
 
// --------------------------------------------------------------------------------------------------------------------//

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
//-----------------------------------------------------meme_interface------------------------------------------//
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

//----------------------------------------------------------memory unit code---------------------------------------//
 
module memory_unit (
        clock, reset, 
        load, store,
        address, 
        store_data,
        data_addr, 
        load_data,
        valid, 
        ready, 
        report
); 

parameter CORE = 0, DATA_WIDTH = 32, INDEX_BITS = 4, 
                     OFFSET_BITS = 3, ADDRESS_BITS = 8;
input clock, reset; 
input load, store;
input [ADDRESS_BITS-1:0] address;
input [DATA_WIDTH-1:0]   store_data;
input report;

output [ADDRESS_BITS-1:0] data_addr;
output [DATA_WIDTH-1:0]   load_data;
output valid; 
output ready;  

mem_interface #(CORE, DATA_WIDTH, INDEX_BITS, OFFSET_BITS, ADDRESS_BITS)  
                    d_mem_interface (
                     .clock(clock), 
                     .reset(reset),
                     .read(load), 
                     .write(store), 
                     .read_address(address), 
                     .write_address(address), 
                     .in_data(store_data), 
                     .out_addr(data_addr),
                     .out_data(load_data), 
                     .valid(valid), 
                     .ready(ready),
                     .report(report)
);

reg [31: 0] cycles; 
always @ (posedge clock) begin 
    cycles <= reset? 0 : cycles + 1; 
    if (report)begin
        $display ("------ Core %d Memory Unit - Current Cycle %d -------", CORE, cycles); 
        $display ("| Address     [%h]", address);
        $display ("| Load        [%b]", load); 
        $display ("| Data Address[%h]", data_addr);
        $display ("| Load Data   [%h]", load_data);
        $display ("| Store       [%b]", store); 
        $display ("| Store Data  [%h]", store_data);
        $display ("| Ready       [%b]", ready);
        $display ("| Valid       [%b]", valid);
        $display ("----------------------------------------------------------------------");
    end
end

endmodule
//------------------------------------------------------------------memeory test banch-------------------------------------//
module tb_memory_unit (); 

reg clk, reset; 

reg load, store; 
reg [7:0] address;
reg [31:0]store_data;
reg report;

wire [31:0]   load_data;
wire [7:0]    data_addr;  
wire valid; 
wire ready; 

//module memory_unit #(parameter CORE = 0, DATA_WIDTH = 32, INDEX_BITS = 6, 
//                     OFFSET_BITS = 3, ADDRESS_BITS = 20)
                     
memory_unit  DM (
        .clock(clk),
        .reset(reset),
        .load(load),
        .store(store),
        .address(address), 
        .store_data(store_data),
        .data_addr(data_addr), 
        .load_data(load_data),
        .valid(valid),
        .ready(ready),
        .report(report)
); 


// Clock generator
always #1 clk = ~clk;

initial begin
  $dumpfile ("memory_unit.vcd");
  $dumpvars();
  clk   = 0;
  reset = 1;
  load  = 0; 
  store = 0; 
  address     = 0; 
  store_data  = 0; 
  report      = 1; 
  
  #10 reset = 0; 
  $display (" --- Start --- ");
  repeat (1) @ (posedge clk);
  
  load  = 0; 
  store = 1; 
  address     = 4; 
  store_data  = 9; 
  repeat (1) @ (posedge clk);
  
  load  = 0; 
  store = 1; 
  address     = 8; 
  store_data  = 5;
  repeat (1) @ (posedge clk);
  
  load  = 1; 
  store = 0; 
  address     = 8; 
  store_data  = 0;
  repeat (1) @ (posedge clk);
  
  load  = 1; 
  store = 1; 
  address     = 12; 
  store_data  = 9;
  repeat (1) @ (posedge clk);
  
  load  = 1; 
  store = 0; 
  address     = 4; 
  store_data  = 0;
  repeat (1) @ (posedge clk);
  #300 $finish;
  end

endmodule
