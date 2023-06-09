/** @module : RISC_V_Core
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
 *
 */

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
parameter CORE = 0, DATA_WIDTH = 32, ADDR_WIDTH = 20;
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
parameter CORE = 0, DATA_WIDTH = 32, INDEX_BITS = 4,
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
                     OFFSET_BITS = 3, ADDRESS_BITS = 20;

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
//----------------------------------------------------------------DECODE UNIT---------------------------------------------//
module regFile (
                clock, reset, read_sel1, read_sel2,
                wEn, write_sel, write_data, 
                read_data1, read_data2
);

parameter REG_DATA_WIDTH = 32, REG_SEL_BITS = 5;

input clock, reset, wEn; 
input [REG_DATA_WIDTH-1:0] write_data;
input [REG_SEL_BITS-1:0] read_sel1, read_sel2, write_sel;
output[REG_DATA_WIDTH-1:0] read_data1; 
output[REG_DATA_WIDTH-1:0] read_data2; 

//(* ram_style = "distributed" *) 
reg   [REG_DATA_WIDTH-1:0] register_file[0:(1<<REG_SEL_BITS)-1];
integer i; 

always @(posedge clock)
    if(reset==1)
        register_file[0] <= 0; 
    else 
        if (wEn & write_sel != 0) register_file[write_sel] <= write_data;
          
//----------------------------------------------------
// Drive the outputs
//----------------------------------------------------
assign  read_data1 = register_file[read_sel1];
assign  read_data2 = register_file[read_sel2];

endmodule

//----------------------------------------------------------------decode unit----------------------------------------//
module decode_unit (
      clock, reset, 
      PC, instruction, 
      extend_sel,
      write, write_reg, write_data, 
      
      opcode, funct3, funct7,
      rs1_data, rs2_data, rd, 
      extend_imm,
      branch_target, 
      JAL_target,  
      report
); 

parameter CORE = 0, ADDRESS_BITS = 20;
 
input  clock; 
input  reset; 

input  [ADDRESS_BITS-1:0] PC;
input  [31:0] instruction; 
input  [1:0] extend_sel; 
input  write;
input  [4:0]  write_reg;
input  [31:0] write_data;

output [31:0] rs1_data; 
output [31:0] rs2_data;
output [4:0]  rd;  
output [6:0]  opcode;
output [6:0]  funct7; 
output [2:0]  funct3;
output [31:0] extend_imm;
output [ADDRESS_BITS-1:0] branch_target; 
output [ADDRESS_BITS-1:0] JAL_target;

input report; 

// Read registers
wire[4:0]  rs2        = instruction[24:20];
wire[4:0]  rs1        = instruction[19:15];

wire[11:0] i_imm      = instruction[31:20];
wire[6:0]  s_imm_msb  = instruction[31:25];
wire[4:0]  s_imm_lsb  = instruction[11:7];
wire[19:0] u_imm      = instruction[31:12];
wire[11:0] i_imm_orig = instruction[31:20]; 
wire[19:0] uj_imm     = {instruction[31],instruction[19:12],instruction[20],instruction[30:21]};

//Forming the s immediate value from the two msb and lsb parts of the s immediate
wire[11:0] s_imm_orig     = {s_imm_msb,s_imm_lsb};
wire[12:0] sb_imm_orig    = {s_imm_msb[6],s_imm_lsb[0],s_imm_msb[5:0],s_imm_lsb[4:1],1'b0};

/* Instruction decoding */
assign opcode        = instruction[6:0];
assign funct7        = instruction[31:25];
assign funct3        = instruction[14:12];

/* Write register */
assign  rd           = instruction[11:7];

/* Only workig with BEQ at the moment */
wire[31:0] sb_imm_32      = {{19{sb_imm_orig[12]}}, sb_imm_orig};
assign branch_target      = PC + sb_imm_32;

/* Extensions */
wire[31:0] u_imm_32       = {u_imm,12'b0};
wire[31:0] i_imm_32       = {{20{i_imm_orig[11]}}, i_imm_orig[11:0] };
wire[31:0] s_imm_32       = {{20{s_imm_orig[11]}}, s_imm_orig};

assign extend_imm         = (extend_sel == 2'b01)? s_imm_32 : 
                            (extend_sel == 2'b10)? u_imm_32 : i_imm_32;
                            
/* Only JAL Target. JALR happens in the execution unit*/
wire[31:0] uj_imm_32      = {{11{uj_imm[19]}},uj_imm[19:0],1'b0}; 
assign JAL_target         = uj_imm_32 + PC;
 
regFile #(32, 5) registers (
                .clock(clock), 
                .reset(reset), 
                .read_sel1(rs1), 
                .read_sel2(rs2),
                .wEn(write), 
                .write_sel(write_reg), 
                .write_data(write_data), 
                .read_data1(rs1_data), 
                .read_data2(rs2_data)
);

reg [31: 0] cycles; 
always @ (posedge clock) begin 
    cycles <= reset? 0 : cycles + 1; 
    if (report)begin
        $display ("------ Core %d Decode Unit - Current Cycle %d -------", CORE, cycles); 
        $display ("| PC          [%h]", PC);
        $display ("| Instruction [%h]", instruction);
        $display ("| rs1         [%d]", rs1);
        $display ("| rs1_data    [%d]", rs1_data);
        $display ("| rs2         [%d]", rs2);
        $display ("| rs2_data    [%d]", rs2_data);
        $display ("| rsd         [%d]", rd);
        $display ("| jumpTarget  [%h]", JAL_target);
        $display ("| branchTarget[%h]", branch_target);
        $display ("| Opcode      [%b]    Funct3 [%b]      Funct7  [%b]", opcode, funct3, funct7);
        $display ("| Immediate   [%d] Hex    [%h] Bin     [%b]", extend_imm, extend_imm, extend_imm[7:0]);
        $display ("| write       [%b]", write);
        $display ("| write_reg   [%d]", write_reg);
        $display ("| write_data  [%d]", write_data);
        $display ("----------------------------------------------------------------------");
    end
end

endmodule
//--------------------------------------EXECUTION UNIT---------------------------------------------------//
module ALU (
        ALU_Control,
        operand_A, operand_B,
        ALU_result, zero, branch
);
 
parameter DATA_WIDTH = 32;
input [5:0] ALU_Control;
input [DATA_WIDTH-1:0]  operand_A ;
input [DATA_WIDTH-1:0]  operand_B ;
output zero, branch;
output [DATA_WIDTH-1:0] ALU_result;


wire [DATA_WIDTH-1:0] signed_operand_A;
wire [DATA_WIDTH-1:0] signed_operand_B;

wire [4:0] shamt = operand_B[4:0];

wire [(DATA_WIDTH*2)-1:0] arithmetic_right_shift_double;
wire [DATA_WIDTH-1:0] arithmetic_right_shift;
wire [DATA_WIDTH-1:0] signed_less_than;
wire [DATA_WIDTH-1:0] signed_greater_than_equal;



//wire signed [DATA_WIDTH-1:0] signed_operand_A;
//wire signed [DATA_WIDTH-1:0] signed_operand_B;

//wire [4:0] shamt = operand_B [4:0];     // I_immediate[4:0];

// wires for signed operations
//wire [(DATA_WIDTH*2)-1:0] arithmetic_right_shift_double;
//wire [DATA_WIDTH-1:0] arithmetic_right_shift;
//wire signed [DATA_WIDTH-1:0] signed_less_than;
//wire signed [DATA_WIDTH-1:0] signed_greater_than_equal;

assign signed_operand_A = operand_A;
assign signed_operand_B = operand_B;

assign zero   = (ALU_result==0);
assign branch = ((ALU_Control[4:3] == 2'b10) & (ALU_result == 1'b1))? 1'b1 : 1'b0;

// Signed Operations
assign arithmetic_right_shift_double = ({ {DATA_WIDTH{operand_A[DATA_WIDTH-1]}}, operand_A }) >> shamt;
assign arithmetic_right_shift = arithmetic_right_shift_double[DATA_WIDTH-1:0];
assign signed_less_than = signed_operand_A < signed_operand_B;
assign signed_greater_than_equal = signed_operand_A >= signed_operand_B;

assign ALU_result =
            (ALU_Control == 6'b000_000)? operand_A + operand_B:                /* ADD, ADDI*/
            (ALU_Control == 6'b001_000)? operand_A - operand_B:                /* SUB */
            (ALU_Control == 6'b000_100)? operand_A ^ operand_B:                /* XOR, XORI*/
            (ALU_Control == 6'b000_110)? operand_A | operand_B:                /* OR, ORI */
            (ALU_Control == 6'b000_111)? operand_A & operand_B:                /* AND, ANDI */
            (ALU_Control == 6'b000_010)? signed_less_than:                     /* SLT, SLTI */
            (ALU_Control == 6'b000_011)? operand_A < operand_B:                /* SLTU, SLTIU */
            (ALU_Control == 6'b000_001)? operand_A << shamt:                   /* SLL, SLLI => 0's shifted in from right */
            (ALU_Control == 6'b000_101)? operand_A >> shamt:                   /* SRL, SRLI => 0's shifted in from left */
            (ALU_Control == 6'b001_101)? arithmetic_right_shift:               /* SRA, SRAI => sign bit shifted in from left */
            (ALU_Control == 6'b011_111)? operand_A:                            /* operand_A = PC+4 for JAL   and JALR */
            (ALU_Control == 6'b010_000)? operand_A == operand_B:               /* BEQ */
            (ALU_Control == 6'b010_001)? operand_A != operand_B:               /* BNE */
            (ALU_Control == 6'b010_100)? signed_less_than:                     /* BLT */
            (ALU_Control == 6'b010_101)? signed_greater_than_equal:            /* BGE */
            (ALU_Control == 6'b010_110)? operand_A < operand_B:                /* BLTU */
            (ALU_Control == 6'b010_111)? operand_A >= operand_B:               /* BGEU */
            {DATA_WIDTH{1'b0}};
endmodule

//---------------------------------------execution_unit------------------------------------//

module execution_unit (
        clock, reset, 
        ALU_Operation, 
        funct3, funct7,
        PC, ALU_ASrc, ALU_BSrc, 
        branch_op, 
        regRead_1, regRead_2, 
        extend,
        ALU_result, zero, branch, 
        JALR_target,    
        
        report
);

parameter CORE = 0, DATA_WIDTH = 32, ADDRESS_BITS = 20;
input  clock; 
input  reset;  
input [2:0] ALU_Operation; 
input [6:0] funct7; 
input [2:0] funct3;
input [ADDRESS_BITS-1:0]  PC;
input [1:0] ALU_ASrc; 
input ALU_BSrc;
input branch_op;
input [DATA_WIDTH-1:0]  regRead_1 ;
input [DATA_WIDTH-1:0]  regRead_2 ; 
input [DATA_WIDTH-1:0]  extend;

output zero, branch; 
output [DATA_WIDTH-1:0] ALU_result;
output [ADDRESS_BITS-1:0] JALR_target;

input report; 
 
wire [5:0] ALU_Control = (ALU_Operation == 3'b011)? 
                         6'b011_111 :      //pass for JAL and JALR
                         (ALU_Operation == 3'b010)? 
                         {3'b010,funct3} : //branches
                         
                         //R Type instructions
                         ({ALU_Operation, funct7} == {3'b000, 7'b0000000})? 
                         {3'b000,funct3} : 
                         ({ALU_Operation, funct7} == {3'b000, 7'b0100000})? 
                         {3'b001,funct3} :
                         (ALU_Operation == 3'b000)?                  
                         {3'b000,funct3} :
                          
                         //I Type instructions
                         ({ALU_Operation, funct3, funct7} == {3'b001, 3'b101, 7'b0000000})? 
                         {3'b000,funct3} : 
                         ({ALU_Operation, funct3, funct7} == {3'b001, 3'b101, 7'b0100000})? 
                         {3'b001,funct3} : 
                         ({ALU_Operation, funct3} == {3'b001, 3'b101})? 
                         {3'b000,funct3} : 
                         (ALU_Operation == 3'b001)?                  
                         {3'b000,funct3} : 
                         6'b000_000;      //addition
                         
wire [DATA_WIDTH-1:0]  operand_A  =  (ALU_ASrc == 2'b01)? PC : 
                                     (ALU_ASrc == 2'b10)? (PC + 4) : regRead_1;
wire [DATA_WIDTH-1:0]  operand_B  =   ALU_BSrc? extend : regRead_2;

wire ALU_branch;
assign branch  = (ALU_branch & branch_op)? 1 : 0; 

ALU #(DATA_WIDTH) EU (
        .ALU_Control(ALU_Control), 
        .operand_A(operand_A), 
        .operand_B(operand_B), 
        .ALU_result(ALU_result), 
        .zero(zero), 
        .branch(ALU_branch)
); 

/* Only JALR Target. JAL happens in the decode unit*/
assign JALR_target        = {regRead_1 + extend} & 32'hffff_fffe; 

reg [31: 0] cycles; 
always @ (posedge clock) begin 
    cycles <= reset? 0 : cycles + 1; 
    if (report)begin
        $display ("------ Core %d Execute Unit - Current Cycle %d ------", CORE, cycles); 
        $display ("| ALU_Operat  [%b]", ALU_Operation);
        $display ("| funct7      [%b]", funct7); 
        $display ("| funct3      [%b]", funct3);
        $display ("| ALU_Control [%b]", ALU_Control);
        $display ("| operand_A   [%h]", operand_A); 
        $display ("| operand_B   [%h]", operand_B);
        $display ("| Zero        [%b]", zero);
        $display ("| Branch      [%b]", branch);
        $display ("| ALU_result  [%h]", ALU_result);
        $display ("| JALR_taget  [%h]", JALR_target);
        $display ("----------------------------------------------------------------------");
    end
end

endmodule
//-----------------------------------------------------------------MEMORY UNIT---------------------------------//
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
                     OFFSET_BITS = 3, ADDRESS_BITS = 20;
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
//----------------------------------------------------WRITEBACK UNIT---------------------------------------------//
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

//------------------------------------------CONTROL UNIT-------------------------------------------------------//
module control_unit (
    clock, reset,
    opcode,
    branch_op, memRead, 
    memtoReg, ALUOp, 
    next_PC_sel, 
    operand_A_sel, operand_B_sel,
    extend_sel,
    memWrite, regWrite, 
    report
);
    parameter CORE = 0;
    input clock;
    input reset;
    input [6:0] opcode;
        
    output branch_op;
    output memRead; 
    output memtoReg; 
    output [2:0] ALUOp; 
    output memWrite;
    output [1:0] next_PC_sel;
    output [1:0] operand_A_sel; 
    output operand_B_sel; 
    output [1:0] extend_sel; 
    output regWrite; 
    input  report;
    
    parameter [6:0]R_TYPE  = 7'b0110011, 
                    I_TYPE  = 7'b0010011, 
                    STORE   = 7'b0100011,
                    LOAD    = 7'b0000011,
                    BRANCH  = 7'b1100011,
                    JALR    = 7'b1100111,
                    JAL     = 7'b1101111,
                    AUIPC   = 7'b0010111,
                    LUI     = 7'b0110111,
                    FENCES  = 7'b0001111,
                    SYSCALL = 7'b1110011;

    assign regWrite      = ((opcode == R_TYPE) | (opcode == I_TYPE) | (opcode == LOAD)
                            | (opcode == JALR) | (opcode == JAL) | (opcode == AUIPC) 
                            | (opcode == LUI))? 1 : 0; 
    assign memWrite      = (opcode == STORE)?   1 : 0; 
    assign branch_op     = (opcode == BRANCH)?  1 : 0; 
    assign memRead       = (opcode == LOAD)?    1 : 0; 
    assign memtoReg      = (opcode == LOAD)?    1 : 0; 

    assign ALUOp         = (opcode == R_TYPE)?  3'b000 : 
                           (opcode == I_TYPE)?  3'b001 :
                           (opcode == STORE)?   3'b101 :   
                           (opcode == LOAD)?    3'b100 : 
                           (opcode == BRANCH)?  3'b010 : 
                           ((opcode == JALR)  | (opcode == JAL))? 3'b011 :
                           ((opcode == AUIPC) | (opcode == LUI))? 3'b110 : 0; 
    
    assign operand_A_sel = (opcode == AUIPC)?  2'b01 : 
                           (opcode == LUI)?    2'b11 : 
                           ((opcode == JALR)  | (opcode == JAL))?  2'b10 : 0; 
                           
    assign operand_B_sel = ((opcode == I_TYPE) | (opcode == STORE)| 
                           (opcode == LOAD) | (opcode == AUIPC) | 
                           (opcode == LUI))? 1 : 0; 

    assign extend_sel    = ((opcode == I_TYPE)  | (opcode == LOAD))?  2'b00 : 
                           (opcode == STORE)?   2'b01  : 
                           ((opcode == AUIPC) | (opcode == LUI))? 2'b10 : 0;

    assign next_PC_sel   = (opcode == BRANCH)?  2'b01 : 
                           (opcode == JAL)?     2'b10 : 
                           (opcode == JALR)?    2'b11 : 0; 
    
reg [31: 0] cycles; 
always @ (posedge clock) begin 
    cycles <= reset? 0 : cycles + 1; 
    if (report)begin
        $display ("------ Core %d Control Unit - Current Cycle %d ------", CORE, cycles); 
        $display ("| Opcode      [%b]", opcode);
        $display ("| Branch_op   [%b]", branch_op);
        $display ("| memRead     [%b]", memRead);
        $display ("| memtoReg    [%b]", memtoReg);
        $display ("| memWrite    [%b]", memWrite);
        $display ("| RegWrite    [%b]", regWrite);
        $display ("| ALUOp       [%b]", ALUOp);
        $display ("| Extend_sel  [%b]", extend_sel);
        $display ("| ALUSrc_A    [%b]", operand_A_sel);
        $display ("| ALUSrc_B    [%b]", operand_B_sel);
        $display ("| Next PC     [%b]", next_PC_sel);
        $display ("----------------------------------------------------------------------");
    end
end
endmodule


//-----------------------------------------RISC_V_CORE_UNIT---------------------------------------------------//

module RISC_V_Core (
    clock,
    reset,
    start,
    prog_address,

    from_peripheral,
    from_peripheral_data,
    from_peripheral_valid,
    to_peripheral,
    to_peripheral_data,
    to_peripheral_valid,

    // In-System Programmer Interface
    isp_address,
    isp_data,
    isp_write,

    current_PC,

    report
);

parameter CORE = 0, DATA_WIDTH = 32, INDEX_BITS = 4,
                     OFFSET_BITS = 3, ADDRESS_BITS = 20;
input  clock, reset, start;
input  [ADDRESS_BITS - 1:0]  prog_address;

// For I/O funstions
input  [1:0]   from_peripheral;
input  [31:0]  from_peripheral_data; 
input          from_peripheral_valid;
output [1:0]   to_peripheral;
output [31:0]  to_peripheral_data; 
output         to_peripheral_valid;

// In-System Programmer Interface
input [ADDRESS_BITS-1:0] isp_address;
input [DATA_WIDTH-1:0] isp_data;
input isp_write;

output [31:0]  current_PC;

input  report; // performance reporting

wire [31:0]  instruction;
wire [ADDRESS_BITS-1: 0] inst_PC;
wire i_valid, i_ready;
wire d_valid, d_ready;

wire [ADDRESS_BITS-1: 0] JAL_target;   
wire [ADDRESS_BITS-1: 0] JALR_target;   
wire [ADDRESS_BITS-1: 0] branch_target; 

wire  write;
wire  [4:0]  write_reg;    
wire  [DATA_WIDTH-1:0] write_data; 

wire [DATA_WIDTH-1:0]  rs1_data; 
wire [DATA_WIDTH-1:0]  rs2_data;
wire [4:0]   rd;  

wire [6:0]  opcode;
wire [6:0]  funct7; 
wire [2:0]  funct3;

wire memRead; 
wire memtoReg;
wire [2:0] ALUOp;
wire branch_op;
wire [1:0] next_PC_sel;
wire [1:0] operand_A_sel; 
wire operand_B_sel; 
wire [1:0] extend_sel; 
wire [DATA_WIDTH-1:0]  extend_imm;
    
wire memWrite;
wire regWrite;

wire branch;
wire [DATA_WIDTH-1:0]   ALU_result; 
wire [ADDRESS_BITS-1:0] generated_addr = ALU_result; // the case the address is not 32-bit

wire ALU_branch; 
wire zero; // Have not done anything with this signal

wire [DATA_WIDTH-1:0]    memory_data;
wire [ADDRESS_BITS-1: 0] memory_addr; // To use to check the address coming out the memory stage

reg  [1:0]   to_peripheral;
reg  [31:0]  to_peripheral_data; 
reg          to_peripheral_valid;

assign current_PC = inst_PC;

fetch_unit #(CORE, DATA_WIDTH, INDEX_BITS, OFFSET_BITS, ADDRESS_BITS) IF (
        .clock(clock),
        .reset(reset),
        .start(start),

        .PC_select(next_PC_sel),
        .program_address(prog_address),
        .JAL_target(JAL_target),
        .JALR_target(JALR_target),
        .branch(branch),
        .branch_target(branch_target),

        .instruction(instruction),
        .inst_PC(inst_PC),
        .valid(i_valid),
        .ready(i_ready),

        .isp_address(isp_address),
        .isp_data(isp_data),
        .isp_write(isp_write),

        .report(report)
);

decode_unit #(CORE, ADDRESS_BITS) ID (
        .clock(clock), 
        .reset(reset),  
        
        .instruction(instruction), 
        .PC(inst_PC),
        .extend_sel(extend_sel),
        .write(write), 
        .write_reg(write_reg), 
        .write_data(write_data), 
      
        .opcode(opcode), 
        .funct3(funct3), 
        .funct7(funct7),
        .rs1_data(rs1_data), 
        .rs2_data(rs2_data), 
        .rd(rd), 
 
        .extend_imm(extend_imm),
        .branch_target(branch_target), 
        .JAL_target(JAL_target),
        
        .report(report)
); 

control_unit #(CORE) CU (
        .clock(clock), 
        .reset(reset),   
        
        .opcode(opcode),
        .branch_op(branch_op), 
        .memRead(memRead), 
        .memtoReg(memtoReg), 
        .ALUOp(ALUOp), 
        .memWrite(memWrite), 
        .next_PC_sel(next_PC_sel), 
        .operand_A_sel(operand_A_sel), 
        .operand_B_sel(operand_B_sel),
        .extend_sel(extend_sel),        
        .regWrite(regWrite), 
        
        .report(report)
);

execution_unit #(CORE, DATA_WIDTH, ADDRESS_BITS) EU (
        .clock(clock), 
        .reset(reset), 
        
        .ALU_Operation(ALUOp), 
        .funct3(funct3), 
        .funct7(funct7),
        .branch_op(branch_op),
        .PC(inst_PC), 
        .ALU_ASrc(operand_A_sel),
        .ALU_BSrc(operand_B_sel),
        .regRead_1(rs1_data), 
        .regRead_2(rs2_data), 
        .extend(extend_imm), 
        .ALU_result(ALU_result), 
        .zero(zero), 
        .branch(branch),
        .JALR_target(JALR_target),
        
        .report(report)
);

memory_unit #(CORE, DATA_WIDTH, INDEX_BITS, OFFSET_BITS, ADDRESS_BITS) MU (
        .clock(clock), 
        .reset(reset), 
        
        .load(memRead), 
        .store(memWrite),
        .address(generated_addr), 
        .store_data(rs2_data),
        .data_addr(memory_addr), 
        .load_data(memory_data),
        .valid(d_valid),
        .ready(d_ready),
        
        .report(report)
); 

writeback_unit #(CORE, DATA_WIDTH) WB (
        .clock(clock), 
        .reset(reset),   
        
        .opWrite(regWrite),
        .opSel(memRead), 
        .opReg(rd), 
        .ALU_Result(ALU_result), 
        .memory_data(memory_data), 
        .write(write), 
        .write_reg(write_reg), 
        .write_data(write_data), 
        
        .report(report)
); 

//Registers s1-s11 [$9,$x18-$x27] are saved across calls ... Using s1-s9 [$9,x18-x25] for final results
always @ (posedge clock) begin        
         if (write && (((write_reg >= 18) && (write_reg <= 25))|| (write_reg == 9)))  begin
              to_peripheral       <= 0;
              to_peripheral_data  <= write_data; 
              to_peripheral_valid <= 1;
              $display (" Core [%d] Register [%d] Value = %d", CORE, write_reg, write_data);
         end
         else to_peripheral_valid <= 0;  
end
    
endmodule

//-----------------------------------------------------testbench RISCV---------------------------//
module tb_RISC_V_Core (); 

reg clock, reset, start; 
reg [19:0] prog_address; 
reg report; // performance reporting


reg [1:0] PC_Select;


reg [7:0] JAL_target;
reg [7:0] JALR_target;
reg branch; 
reg [7:0] branch_target;

wire [31:0]   instruction;
wire [7:0]    inst_PC;  
wire valid; 
wire ready; 

// module RISC_V_Core #(parameter CORE = 0, DATA_WIDTH = 32, INDEX_BITS = 6, OFFSET_BITS = 3, ADDRESS_BITS = 20)
RISC_V_Core CORE (
                .clock(clock), 
                .reset(reset), 
                .start(start), 
                .prog_address(prog_address), 
                
                .from_peripheral(),
                .from_peripheral_data(), 
                .from_peripheral_valid(), 
                .to_peripheral(),
                .to_peripheral_data(), 
                .to_peripheral_valid(),
                
                .report(report)
); 


// Clock generator
always #1 clock = ~clock;

initial begin
  
  clock  = 0;
  reset  = 1;
  report = 0; 
  prog_address = 'h04; 
  repeat (1) @ (posedge clock);
  reset = 0;
  start = 1; 
  PC_Select<= 1;
 
  repeat (1) @ (posedge clock);

  start = 0; 
  repeat (1) @ (posedge clock);   
  #200 $finish;     
end

endmodule
