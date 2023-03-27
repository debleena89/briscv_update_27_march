/** @module : execute
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
 
// 32-bit Exection 


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
//-----------------------------------------------------test bench for exevute-----------------------------------------------------------------//
module tb_execution_unit (); 

reg  clk, reset;  
reg [2:0] ALU_Operation; 
reg [6:0] funct7; 
reg [2:0] funct3;
reg [19:0]  PC;
reg [1:0] ALU_ASrc; 
reg ALU_BSrc;
reg branch_op;
reg [31:0]  regRead_1 ;
reg [31:0]  regRead_2 ; 
reg [31:0]  extend;

wire zero, branch; 
wire [31:0] ALU_result;
wire [19:0]JALR_target;

reg report; 

execution_unit execute (
	.clock(clk),
	.reset(reset), 
	.ALU_Operation(ALU_Operation), 
	.funct3(funct3),
        .funct7(funct7),
	.PC(PC),
	.ALU_ASrc(ALU_ASrc),
	.ALU_BSrc(ALU_BSrc),
	.branch_op(branch_op),	
	.regRead_1(regRead_1),
	.regRead_2(regRead_2), 
	.extend(extend),
	.ALU_result(ALU_result),
	.zero(zero),
	.branch(branch), 
	.JALR_target(JALR_taget),	
	.report(report)
);

// Clock generator
always #1 clk = ~clk;

initial begin
  $dumpfile ("execute.vcd");
  $dumpvars();
  clk   = 0;
  reset = 1;
  ALU_Operation = 0; 
  funct3        = 0; 
  funct7        = 0; 
  branch_op     = 0; 
  regRead_1     = 0; 
  regRead_2     = 0; 
  report        = 1; 
  
  #10 reset = 0; 
  $display (" --- Start --- ");
  repeat (1) @ (posedge clk);
  
  ALU_Operation <= 3'b000; 
  funct3        <= 3'b101; 
  funct7        <= 7'b0000000; 
  regRead_1     <= 5; 
  regRead_2     <= 7; 
  repeat (1) @ (posedge clk);
  
  ALU_Operation <= 3'b000; 
  funct3        <= 3'b000; 
  funct7        <= 7'b0100000; 
  regRead_1     <= 5; 
  regRead_2     <= 7; 
  repeat (1) @ (posedge clk);
  
  ALU_Operation <= 3'b001; 
  funct3        <= 3'b000; 
  funct7        <= 7'b0000000; 
  regRead_1     <= 5; 
  regRead_2     <= 7;
  repeat (1) @ (posedge clk);
  #200 $finish;
  end

endmodule
