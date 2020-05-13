/* verilator lint_off STMTDLY */

`define ALU_ADD 3'b000
`define ALU_SLT 3'b010
`define ALU_SLTU 3'b011
`define ALU_XOR 3'b100
`define ALU_OR 3'b110
`define ALU_AND 3'b111
`define ALU_SLL 3'b001
`define ALU_SRL 3'b101

`define ALU_A_SEL_PC 1'b1
`define ALU_B_SEL_IMM 1'b1

`define REGFILE_WB_B 3'b000
`define REGFILE_WB_BU 3'b100
`define REGFILE_WB_H 3'b001
`define REGFILE_WB_HU 3'b101
`define REGFILE_WB_W 3'b010
`define REGFILE_WB_NO 3'b011

`define REGFILE_RDSEL_ALU 2'b00
`define REGFILE_RDSEL_PCP4 2'b01
`define REGFILE_RDSEL_MEM 2'b10
`define REGFILE_RDSEL_IMM 2'b11

`define COMPUNIT_EQ 3'b000
`define COMPUNIT_NE 3'b001
`define COMPUNIT_LT 3'b100
`define COMPUNIT_GE 3'b101
`define COMPUNIT_LTU 3'b110
`define COMPUNIT_GEU 3'b111
`define COMPUNIT_TRUE 3'b010
`define COMPUNIT_FALSE 3'b011

`define MEM_ADDR_SEL_PC 1'b0

`define IMM_ZERO_TYPE 3'b000
`define IMM_I_TYPE 3'b001
`define IMM_S_TYPE 3'b010
`define IMM_B_TYPE 3'b011
`define IMM_U_TYPE 3'b100
`define IMM_J_TYPE 3'b101

/***************************************************************
    IDecoder

    The IDecoder is used to extract all control signals
    from the instruction word
***************************************************************/

module IDecoder(
    input [6:0] op,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg ir_ret,

    //==ALU Control outputs
    output reg ALU_A_sel,
    output reg ALU_B_sel,
    output reg [2:0] ALU_func,
    output reg ALU_func_mod,    

    //==CompUnit Control output regs
    output reg [2:0] CompUnit_cond,

    //==RegFile Control output regs
    output reg RegFile_wb_en,

    output reg [2:0] RegFile_wb_mode,
    output reg [1:0] RegFile_rd_sel,

    //==Control information
    output reg mem_inst,

    //==Mem write enable  
    output reg [3:0] Mem_we,
 
    input clk,
    input nreset
);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ALU Output
always @ (posedge clk, negedge nreset)
begin
    if( ~nreset ) begin
            ALU_A_sel <= 0;
            ALU_B_sel <= 0;
            ALU_func <= 0;
            ALU_func_mod <= 0;
            CompUnit_cond <= 0;
            RegFile_wb_en <= 0;
            RegFile_wb_mode <= 0;
            RegFile_rd_sel <= 0;
            mem_inst <= 0;
            Mem_we <= 0;
            ir_ret <= 0; 
    end
    else begin
        if( op == 7'b1110011 && funct3 == 0 && funct7 == 7'b0011000 ) begin
            //Return
            ALU_A_sel <= ~`ALU_A_SEL_PC;
            ALU_B_sel <= ~op[5];
            //--ALU function
            ALU_func <= funct3;
            ALU_func_mod <= op[5] ? funct7[5] : (funct3 == 3'b101) & funct7[5];
            //--CompUnit condition
            CompUnit_cond <= `COMPUNIT_FALSE;
            //--RegFile Control outputs
            RegFile_wb_mode <= `REGFILE_WB_W;
            RegFile_rd_sel <= `REGFILE_RDSEL_ALU;
            RegFile_wb_en <= 0;
            //==Control information
            mem_inst <= 0;

            //==Mem WE
            Mem_we <= 0;

            ir_ret <= 1;
        end
        else if( ~op[6] && op[4:0] == 5'b10011  )begin 
            //RR or RI Arithmetic
            //--ALU operands select
            ALU_A_sel <= ~`ALU_A_SEL_PC;
            ALU_B_sel <= ~op[5];
            //--ALU function
            ALU_func <= funct3;
            ALU_func_mod <= op[5] ? funct7[5] : (funct3 == 3'b101) & funct7[5];
            //--CompUnit condition
            CompUnit_cond <= `COMPUNIT_FALSE;
            //--RegFile Control outputs
            RegFile_wb_mode <= `REGFILE_WB_W;
            RegFile_rd_sel <= `REGFILE_RDSEL_ALU;
            RegFile_wb_en <= 1;
            //==Control information
            mem_inst <= 0;

            //==Mem WE
            Mem_we <= 0;

            ir_ret <= 0; 
        end
        else if( ~op[6] && op[4:0] == 5'b00011 ) begin
            //Store or Load
            //--ALU operands select
            ALU_A_sel <= ~`ALU_A_SEL_PC;
            ALU_B_sel <= `ALU_B_SEL_IMM;
            //--ALU function
            ALU_func <= `ALU_ADD;
            ALU_func_mod <= 0;
            //--CompUnit condition
            CompUnit_cond <= `COMPUNIT_FALSE;
            //--RegFile Control outputs
            RegFile_wb_en <= ~op[5];
            RegFile_wb_mode <= funct3;
            RegFile_rd_sel <= `REGFILE_RDSEL_MEM;
            
            //==Control information
            mem_inst <= 1;

            if( ~op[5] ) begin
                Mem_we <= 0;
            end
            else begin
                case( funct3 )
                    3'b000:
                        Mem_we <= 4'b0001;
                    3'b001:
                        Mem_we <= 4'b0011;
                    3'b010:
                        Mem_we <= 4'b1111;
                    default: begin
                        Mem_we <= 0;
                    end
                endcase
            end

            ir_ret <= 0; 
        end
        else if( op[6:4] == 3'b110 && op[1:0] == 2'b11 ) begin
            //Branch and Jump Instruction
            //--ALU operands select
            ALU_A_sel <= op[3] | ~op[2] ? `ALU_A_SEL_PC : ~`ALU_A_SEL_PC; 
            ALU_B_sel <= `ALU_B_SEL_IMM;
            //--ALU function
            ALU_func <= `ALU_ADD;
            ALU_func_mod <= 0;
            //--CompUnit condition
            CompUnit_cond <= op[2] ? `COMPUNIT_TRUE : funct3;
            //--RegFile Control outputs
            RegFile_wb_en <= op[2];
            RegFile_wb_mode <= `REGFILE_WB_W;
            RegFile_rd_sel <= `REGFILE_RDSEL_PCP4;

            //==Control information
            mem_inst <= 0;

            //==Mem WE
            Mem_we <= 0;


            ir_ret <= 0; 
        end
        else if( ~op[6] && op[4:0] == 5'b10111 ) begin
            //LUI and AUIPC
            //--ALU operands select
            ALU_A_sel <= `ALU_A_SEL_PC;
            ALU_B_sel <= `ALU_B_SEL_IMM;
            //--ALU function
            ALU_func <= `ALU_ADD;
            ALU_func_mod <= 0;
            //--CompUnit condition
            CompUnit_cond <= `COMPUNIT_FALSE;
            //--RegFile Control outputs
            RegFile_wb_en <= 1;
            RegFile_wb_mode <= `REGFILE_WB_W;
            RegFile_rd_sel <= op[5] ? `REGFILE_RDSEL_IMM : `REGFILE_RDSEL_ALU;

            //==Control information
            mem_inst <= 0;

            //==Mem WE
            Mem_we <= 0;

            ir_ret <= 0; 
        end
        else begin
            ALU_A_sel <= 0;
            ALU_B_sel <= 0;
            ALU_func <= 0;
            ALU_func_mod <= 0;
            CompUnit_cond <= 0;
            RegFile_wb_en <= 0;
            RegFile_wb_mode <= 0;
            RegFile_rd_sel <= 0;
            mem_inst <= 0;
            Mem_we <= 0;

            ir_ret <= 0; 
        end
    end    
end

endmodule

/*****************************************************************
    ImmDecoder

    The ImmDecoder is used to extract the immediate from teh
    instruction word
*****************************************************************/

module ImmDecoder(
    input [31:0] inst,
    output reg [31:0] Imm,
    input ID,
    input clk,
    input nreset
);

reg [2:0] imm_type;
wire [6:0] op = inst[6:0];

always @ (*)
begin
    if( ~op[6] && op[4:0] == 5'b10011  )begin 
        //==ImmDecoder information
        imm_type = `IMM_I_TYPE;
    end
    else if( ~op[6] && op[4:0] == 5'b00011 ) begin
        //==ImmDecoder information
        imm_type = op[5] ? `IMM_S_TYPE : `IMM_I_TYPE;
    end
    else if( op[6:4] == 3'b110 && op[1:0] == 2'b11 ) begin
        //==ImmDecoder information
        case( op[3:2] )
            2'b11:
                imm_type = `IMM_J_TYPE;
            2'b01:
                imm_type = `IMM_I_TYPE;
            2'b00:
                imm_type = `IMM_B_TYPE;
            default:
                imm_type = `IMM_ZERO_TYPE;
        endcase
    end
    else if( ~op[6] && op[4:0] == 5'b10111 ) begin
        //==ImmDecoder information
        imm_type = `IMM_U_TYPE;
    end
    else begin
        imm_type = `IMM_ZERO_TYPE;
    end
end

always @( posedge clk, negedge nreset )
begin
    if( ~nreset ) begin
        Imm <= 0;
    end
    else begin
        if( ID ) begin
            case( imm_type )
                `IMM_ZERO_TYPE:
                    Imm <= 0;    
                `IMM_I_TYPE:
                    Imm <= {{20{inst[31]}},inst[31:20]};
                `IMM_S_TYPE:
                    Imm <= {{20{inst[31]}},inst[31:25],inst[11:7]};
                `IMM_B_TYPE:
                    Imm <= {{20{inst[31]}},inst[7],inst[30:25],inst[11:8],1'b0};
                `IMM_U_TYPE:
                    Imm <= {inst[31:12], 12'h000 };
                `IMM_J_TYPE:
                    Imm <= {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21],1'b0 };
                default:
                    Imm <= 0;
            endcase
        end
        else Imm <= Imm;
    end
end

endmodule
`timescale 1ns/1ps
module Control(
    //==Control Input from IDecoder
    input mem_inst,
    input ir_request,
    output reg ir_grant,
    input ir_ret,

    //==Register we signals
    output IReg_we,
    output reg PC_we,

    output reg ePC_we,
    output reg load_ivec,

    //==Bus Signals
    output reg valid,
    input ready,

    //==Stage Controls
    output IF,
    output ID1,
    output ID2,
    output EXE,
    output MEM,
    output WB,

    //==Sync Signals
    input clk,
    input nreset
);

localparam IF_state =  7'b0000001;
localparam ID1_state = 7'b0000010;
localparam ID2_state = 7'b0000100;
localparam EXE_state = 7'b0001000;
localparam MEM_state = 7'b0010000;
localparam WB_state =  7'b0100000;
localparam IR_state =  7'b1000000;

reg [6:0] state;

assign IF = state[0];
assign ID1 = state[1];
assign ID2 = state[2];
assign EXE = state[3];
assign MEM = state[4];
assign WB = state[5];
wire IR = state[6];

always @ (posedge clk, negedge nreset )
begin
    if( ~nreset ) begin
        state <= IF_state;
        valid <= 0;
        PC_we <= 0;
        ePC_we <= 0;
        load_ivec <= 0;
    end
    else begin
        
        case( state )
            IF_state: begin
                if( ready ) begin
                    valid <= 1'b0;
                    state <= ID1_state;
                end
                else begin
                    valid <= 1'b1;
                    state <= IF_state;
                end
            end
                
            ID1_state: begin
                valid <= 0;
                state <= ID2_state;
            end

            ID2_state: begin
                state <= EXE_state;
            end

            EXE_state: begin
                state <= MEM_state;
                //valid <= mem_inst ? 1 : 0;
            end

            MEM_state: begin
                if( ~mem_inst | ready) begin
                    PC_we <= 1;
                    valid <= 0;
                    state <= WB_state;
                end
                else begin
                    PC_we <= 0;
                    valid <= 1;
                    state <= MEM_state;
                end
                /*
                if( ready | ~mem_inst ) begin
                    PC_we <= 1;
                    valid <= 0;
                    state <= WB_state;
                end
                else begin
                    PC_we <= 0;
                    valid <= 1;
                    state <= MEM_state;
                end
                */
            end

            WB_state: begin
                PC_we <= 0;
                if( ir_request && !ir_grant ) begin
                    state <= IR_state;
                    ePC_we <= 1;
                    load_ivec <= 1;
                end
                else state <= IF_state;
                //valid <= 1'b1;
            end
            
            IR_state: begin
                ePC_we <= 1'b0;
                load_ivec <= 1'b0;
                state <= IF_state;
            end

            default:
                state <= IF_state;
        endcase
    end
end

assign IReg_we = IF & valid & ready;

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        ir_grant <= 0;
    end
    else begin
        if( ir_grant && IF ) ir_grant <= ir_ret ? 0 : 1;
        else if( ir_request && IR ) ir_grant <= 1;
    end
end

endmodule
/****************************************************************
    ALU
    
    function: 
    The ALU is used to calculate all major arithmetic 
    operations like RR, RI or PC Arithmetic.
    For every operation the ALU takes one clock cycle

****************************************************************/
module ALU(
	input [2:0] func,
    input mod,
    input [31:0] rs1_data,
    input [31:0] rs2_data,
    input [31:0] PC,
    input [31:0] Imm,

	input A_sel,
	input B_sel,
	output reg [31:0] C,
    

    input execute,
    input clk,
    input nreset
);

wire [31:0] A = A_sel ? PC : rs1_data;
wire [31:0] B = B_sel ? Imm : rs2_data;

always @ ( posedge clk, negedge nreset )
begin
    if( ~nreset ) begin
        C <= 0;
    end
    else begin
        if( execute ) begin
            case( func ) 
                `ALU_ADD:
                    C <= mod ? A-B : A+B;
                `ALU_SLT:
                    C <= $signed(A) < $signed(B) ? 1 : 0;
                `ALU_SLTU:
                    C <= A < B ? 1 : 0;
                `ALU_XOR:
                    C <= A^B;
                `ALU_OR:
                    C <= A|B;
                `ALU_AND:
                    C <= A&B;
                `ALU_SLL:
                    C <= A << B[4:0];
                `ALU_SRL:
                    if(mod)
                        C <= $signed(A) >>> B[4:0];
                    else
                        C <= A >> B[4:0];
                default:
                    C <= 0;
            endcase
        end
        else C <= C;
    end
end

endmodule
/***********************************************************
    CompUnit

    The CompUnit is used to determite the relation between
    two values and activly determies if a jump is taken
************************************************************/

module CompUnit(
	input [31:0] A,
	input [31:0] B,
	input [2:0] cond,
	output reg met,
    input execute,
    input clk,
    input nreset
);


wire signed [31:0] A_signed = $signed(A);
wire signed [31:0] B_signed = $signed(B);

always @ (posedge clk, negedge nreset )
begin
    if( ~nreset ) begin
        met <= 0;
    end
    else begin
        if( execute ) begin
            case( cond )
                `COMPUNIT_TRUE:
                    met <= 1'b1;
                `COMPUNIT_FALSE:
                    met <= 1'b0;
                `COMPUNIT_EQ:
                    met <= A == B ? 1'b1 : 1'b0;
                `COMPUNIT_NE:
                    met <= A != B ? 1'b1 : 1'b0;
                `COMPUNIT_LT:
                    met <= A_signed < B_signed ? 1'b1 : 1'b0;
                `COMPUNIT_GE:
                    met <= A_signed >= B_signed ? 1'b1 : 1'b0;
                `COMPUNIT_LTU:
                    met <= A < B ? 1'b1 : 1'b0;
                `COMPUNIT_GEU:
                    met <= A >= B ? 1'b1 : 1'b0;
                default:
                    met <= 0;
            endcase
        end
        else met <= met;
    end
end

endmodule
module Rf2p_32x32_t90(
    //==Read Port A
    input [4:0] AA,
    output reg [31:0] QA,
    input CLKA,
    input CENA,

    //==Write Port B
    input [4:0] AB,
    input [31:0] DB,
    input CENB,
    input CLKB,

    //==Self Adjusg
    input [2:0] EMAA,
    input [2:0] EMAB
);

reg [31:0] memory [0:31];

//==Read Process
always @(posedge CLKA)
begin
    if( !CENA ) begin
        QA <= memory[AA];
    end
end

//==Write Process
always @(posedge CLKB)
begin
    if( !CENB ) begin
        memory[AB] <= DB;
    end
end

endmodule
/********************************************************************
    DataPath

    The DataPath is the core component of the data processing
    stream. In the DataPath the next pc is determined, mem is accessed
    and register are written
********************************************************************/

module DataPath(
    //==Extern Operands
    input [31:0] PC,
    input [31:0] Imm,
   
    //==RegFile Control Signals
    input [4:0] RegFile_rs1,
    input [4:0] RegFile_rs2,
    input [4:0] RegFile_rd,
    input RegFile_wb_en,
    input [2:0] RegFile_wb_mode,    
    input [1:0] RegFile_rd_sel,

    //==ALU Control Signals 
    input [2:0] ALU_func,
    input ALU_mod,
    input ALU_A_sel,
    input ALU_B_sel,

    //==CompUnit Control Signals
    input [2:0] CompUnit_cond,

    //==Generated Signals
    output reg [31:0] Next_PC,
    output [31:0] Addr,
    output [31:0] Dout,
    input [31:0] Din,
    input [3:0] we,
    output reg [3:0] we_alligned,

    //==Control Signals and Phases
    input IF,
    input ID1,
    input ID2,
    input EXE,
    input MEM,
    input WB,

    input clk,
    input nreset
);

/////////////////////////////////////////////////////////////////
//Helper Function
function [31:0] sign_extd( 
    input [31:0] x,
    input [2:0] m
);
begin
    case(m)
        `REGFILE_WB_B: 
            sign_extd = {{24{x[7]}}, x[7:0] };
        `REGFILE_WB_BU:
            sign_extd = {{24{1'b0 }}, x[7:0] };
        `REGFILE_WB_H:
            sign_extd = {{16{x[15]}}, x[15:0] };
        `REGFILE_WB_HU:
            sign_extd = {{16{1'b0}}, x[15:0] };
        `REGFILE_WB_W: 
            sign_extd = x;
        default:
            sign_extd = x;
    endcase
end
endfunction

/////////////////////////////////////////////////////////////////
//Connection Signals
wire [31:0] RegFile_rd_data;
reg [31:0] RegFile_rs1_data;
wire [31:0] RegFile_rs2_data;
wire [31:0] RegFile_DB;
wire [4:0] RegFile_AA;
wire [31:0] RegFile_QA;
wire RegFile_CENA;
wire RegFile_CENB;

wire [31:0] ALU_C;

wire CompUnit_met;

reg [31:0] PC_p4;

reg [31:0] Din_alligned;
reg [31:0] Dout_alligned;


always @*
begin
    case( ALU_C[1:0] )
        2'b00: Dout_alligned = RegFile_rs2_data;
        2'b01: Dout_alligned = {RegFile_rs2_data[23:0], 8'h0};
        2'b10: Dout_alligned = {RegFile_rs2_data[15:0], 16'h0};
        2'b11: Dout_alligned = {RegFile_rs2_data[7:0], 24'h0};
    endcase
end

always @*
begin
    case( ALU_C[1:0] )
        2'b00: Din_alligned = Din;
        2'b01: Din_alligned = {8'h0, Din[31:8]};
        2'b10: Din_alligned = {16'h0, Din[31:16]};
        2'b11: Din_alligned = {24'h0, Din[31:24]};
    endcase
end

wire [2:0] byte_addr = RegFile_rs1_data[1:0] + Imm[1:0];

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        we_alligned <= 0;
    end
    else begin
        if( IF ) begin
            we_alligned <= 4'b00;
        end
        if( EXE ) begin
            case( byte_addr[1:0] ) 
                2'b00: we_alligned <= we;
                2'b01: we_alligned <= {we[2:0], 1'b0};
                2'b10: we_alligned <= {we[1:0], 2'b00};
                2'b11: we_alligned <= {we[0], 3'b000};
            endcase
        end
    end
end


/////////////////////////////////////////////////////////////////
//Static Assignments
//FIXME
//assign Dout = RegFile_rs2_data;
assign Dout = Dout_alligned;
assign Addr = ALU_C[31:0];
/*
always @ (posedge clk, negedge nreset)
begin
    if( ~nreset ) begin
        RegFile_rd_data <= 0;
    end
    else begin
        if( WB && RegFile_rd != 0 && RegFile_wb_en ) begin
            case( RegFile_rd_sel )
                REGFILE_RDSEL_ALU:
                    RegFile_rd_data <= ALU_C;
                REGFILE_RDSEL_IMM:
                    RegFile_rd_data <= Imm;
                REGFILE_RDSEL_PCP4:
                    RegFile_rd_data <= PC_p4;
                REGFILE_RDSEL_MEM:
                    RegFile_rd_data <= Din;
                default:
                    RegFile_rd_data <= 0;
            endcase
        end
        else RegFile_rd_data <= 0;
    end
end
*/
/*
always @*
begin
    if( RegFile_rd != 0 && RegFile_wb_en ) begin
        case( RegFile_rd_sel )
            REGFILE_RDSEL_ALU:
                RegFile_rd_data <= ALU_C;
            REGFILE_RDSEL_IMM:
                RegFile_rd_data <= Imm;
            REGFILE_RDSEL_PCP4:
                RegFile_rd_data <= PC_p4;
            REGFILE_RDSEL_MEM:
                RegFile_rd_data <= Din;
            default:
                RegFile_rd_data <= 0;
        endcase
    end
    else RegFile_rd_data <= 0;
end
*/
assign RegFile_rd_data = RegFile_rd == 0 || !RegFile_wb_en ? 0 :
                        RegFile_rd_sel == `REGFILE_RDSEL_ALU ? ALU_C :
                        RegFile_rd_sel == `REGFILE_RDSEL_IMM ? Imm :
                        RegFile_rd_sel == `REGFILE_RDSEL_PCP4 ? PC_p4 :
                            //FIXME
                        //RegFile_rd_sel == REGFILE_RDSEL_MEM ? Din :
                        RegFile_rd_sel == `REGFILE_RDSEL_MEM ? Din_alligned :
                        0;

assign RegFile_DB = sign_extd( RegFile_rd_data, RegFile_wb_mode );

/////////////////////////////////////////////////////////////////
//Registers
always @ (posedge clk, negedge nreset )
begin
    if( ~nreset ) begin
        PC_p4 <= 0;
    end
    else begin
        if( EXE ) begin
            PC_p4 <= PC + 4;
        end
        else begin
            PC_p4 <= PC_p4;
        end
    end
end

always @ (posedge clk, negedge nreset )
begin
    if( ~nreset ) begin
        Next_PC <= 0;
    end
    else begin
        if( MEM ) begin
            Next_PC <= CompUnit_met ? {ALU_C[31:1],1'b0} : PC_p4;
        end
        else begin
            Next_PC <= Next_PC;
        end
    end
end
//////////////////////////////////////////////////////////////////
// New RegFile Adapter code
assign RegFile_AA = ID1 ? RegFile_rs1 : RegFile_rs2;

always @(posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        RegFile_rs1_data <= 32'h0;
    end
    else begin
        if( ID2 ) begin
            RegFile_rs1_data <= (RegFile_rs1 == 5'h0) ? 32'h0 : RegFile_QA;
        end
    end
end

assign RegFile_rs2_data = (RegFile_rs2 == 5'h0) ? 32'h0 : RegFile_QA;

assign RegFile_CENA = ~(ID1 | ID2);
assign RegFile_CENB = ~(RegFile_wb_en & WB);
/////////////////////////////////////////////////////////////////
//Instances 

Rf2p_32x32_t90 regFile(
    //==Read Port A
    .AA( RegFile_AA ),
    .QA( RegFile_QA ),
    .CLKA( clk ),
    .CENA( RegFile_CENA ),

    //==Write Port B
    .AB( RegFile_rd ),
    .DB( RegFile_DB ),
    .CENB( RegFile_CENB ),
    .CLKB( clk ),

    //==Self Adjusg
    .EMAA( 3'b0 ),
    .EMAB( 3'b0 )
);

ALU alu(
	.func( ALU_func ),
    .mod( ALU_mod ),
    .rs1_data( RegFile_rs1_data ),
    .rs2_data( RegFile_rs2_data ),
    .PC( PC ),
    .Imm( Imm ),

	.A_sel( ALU_A_sel ),
	.B_sel( ALU_B_sel ),
	.C( ALU_C ),

    .execute( EXE ),
    .clk( clk ),
    .nreset( nreset )
);

CompUnit compUnit(
	.A( RegFile_rs1_data ),
	.B( RegFile_rs2_data ),
	.cond( CompUnit_cond ),
	.met( CompUnit_met ),
    .execute( EXE ),
    .clk( clk ),
    .nreset( nreset )
);
endmodule
`ifdef SIMULATION
`timescale 1ns/1ps
`endif

/*******************************************************************
Core Module:

The core is the main component of the Processor and incorpartes

==Sub Modules==

    +IDecoder
    +ImmDecoder
    +Control
    +Datapath

==Parameters==

    +Reset_Addr : 32 bit Value
        function: Gives the address, the core jumps to right 
                  after a reset

        defaultl: 0

*******************************************************************/
module Core #
(
    parameter RESET_ADDR = 32'h00000000
)
(
    //==System Bus
    output valid,
    input ready,
    output [3:0] we,
    output reg [31:0] Addr,
    output reg [31:0] Dout,
    input [31:0] Din,

    input ir_request,
    output ir_grant,
    input [31:0] ivec,

    //==Sync Signals
    input clk,
    input nreset
);

///////////////////////////////////////////////////////////
//Connection Signals
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Control
wire IReg_we;
wire PC_we;
wire ePC_we;
wire ir_ret;
wire load_ivec;
wire IF;
wire ID1;
wire ID2;
wire EXE;
wire MEM;
wire WB;
reg ir_request_r;
reg [31:0] ivec_r;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//DataPath
wire [31:0] Next_PC;
wire [31:0] DataPath_Dout;
wire [31:0] DataPath_Addr;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ImmDecoder
wire [31:0] Imm;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//IDecoder
wire ALU_A_sel;
wire ALU_B_sel;
wire [2:0] ALU_func;
wire ALU_mod;    
wire [2:0] CompUnit_cond;
wire RegFile_wb_en;
wire RegFile_shadow_ra_we;
wire [2:0] RegFile_wb_mode;
wire [1:0] RegFile_rd_sel;
wire mem_inst;
wire [3:0] Mem_we;

///////////////////////////////////////////////////////////
//Registers and Mux Definition
reg [31:0] IReg;
reg [31:0] PC;
reg [31:0] ePC;
reg [31:0] Din_reg;

///////////////////////////////////////////////////////////
//Static Assignments
wire [6:0] op = IReg[6:0];
wire [2:0] funct3 = IReg[14:12];
wire [6:0] funct7 = IReg[31:25];
wire [4:0] RegFile_rs1 = IReg[19:15];
wire [4:0] RegFile_rs2 = IReg[24:20];
wire [4:0] RegFile_rd = IReg[11:7];

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Register and Mux logic
always @ (posedge clk, negedge nreset )
begin
    if( ~nreset ) begin
        IReg <= 0;
    end
    else begin
        if( IReg_we ) begin
            //jalr ra, x0, 0x8 for interrupt
            IReg <= Din;
        end
    end
end

always @ (posedge clk, negedge nreset )
begin
    if( ~nreset ) begin
        PC <= RESET_ADDR;
    end
    else begin
        if( load_ivec ) PC <= ivec_r;
        else if( PC_we ) begin
            PC <= ir_ret ? ePC : Next_PC;
        end
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) ePC <= 0;
    else if( ePC_we ) ePC <= PC;
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        ir_request_r <= 0;
        ivec_r <= 0;
    end
    else begin
        if( MEM ) begin
            ir_request_r <= ir_request;
            ivec_r <= ivec;
        end
    end
end
//assign we = IF ? 4'b0000 : Mem_we;
//FIXME
/*
assign we = IF || ID ? 4'b0000 : 
            Addr[1:0] == 2'b00 ? Mem_we : 
            Addr[1:0] == 2'b01 ? {Mem_we[2:0],1'b0} :
            Addr[1:0] == 2'b10 ? {Mem_we[1:0],2'b00} :
            {Mem_we[0], 3'b000};
*/


always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        Din_reg <= 0;
    end
    else begin
        if( (MEM || IF) && valid && ready ) Din_reg <= Din;
        else Din_reg <= Din_reg;
    end
end

always @ (posedge clk, negedge nreset )
begin
    if( !nreset ) Addr <= 0;
    else if( IF ) Addr <= PC;
    else if( MEM && mem_inst ) Addr <= DataPath_Addr;
    else Addr <= 0;
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        Dout <= 0;
    end
    else begin
        if( MEM && mem_inst && |we ) begin
            Dout <= DataPath_Dout;
        end
        else begin
            Dout <= 0;
        end
    end
end

///////////////////////////////////////////////////////////
//Instances
Control control(
    //==Control Input from IDecoder
    .mem_inst( mem_inst ),

    //==Register we signals
    .IReg_we( IReg_we ),
    .PC_we( PC_we ),
    .ePC_we( ePC_we ),
    .load_ivec( load_ivec ),

    //==Bus Signals
    .valid( valid ),
    .ready( ready ),

    .ir_request( ir_request_r ),
    .ir_grant( ir_grant ),
    .ir_ret( ir_ret ),
    //==Stage Controls
    .IF( IF ),
    .ID1( ID1 ),
    .ID2( ID2 ),
    .EXE( EXE ),
    .MEM( MEM ),
    .WB( WB ),

    //==Sync Signals
    .clk( clk ),
    .nreset( nreset )
);

DataPath dataPath(
    //==Extern Operands
    .PC( PC ),
    .Imm( Imm ),
   
    //==RegFile Control Signals
    .RegFile_rs1( RegFile_rs1 ),
    .RegFile_rs2( RegFile_rs2 ),
    .RegFile_rd( RegFile_rd ),
    .RegFile_wb_en( RegFile_wb_en ),
    .RegFile_wb_mode( RegFile_wb_mode ),
    .RegFile_rd_sel( RegFile_rd_sel ),

    //==ALU Control Signals 
    .ALU_func( ALU_func ),
    .ALU_mod( ALU_mod ),
    .ALU_A_sel( ALU_A_sel ),
    .ALU_B_sel( ALU_B_sel ),

    //==CompUnit Control Signals
    .CompUnit_cond( CompUnit_cond ),

    //==Generated Signals
    .Next_PC( Next_PC ),
    .Addr( DataPath_Addr ),
    .Dout( DataPath_Dout ),
    .Din( Din_reg ),
    .we( Mem_we ),
    .we_alligned( we ),
    //==Control Signals and Phases
    .IF( IF ),
    .ID1( ID1 ),
    .ID2( ID2 ),
    .EXE( EXE ),
    .MEM( MEM ),
    .WB( WB ),

    .clk( clk ),
    .nreset( nreset )
);

ImmDecoder immDecoder(
    .inst( IReg ),
    .Imm( Imm ),
    .ID( ID1 ),
    .clk( clk ),
    .nreset( nreset )
);

IDecoder iDecoder(
    .op( op ),
    .funct3( funct3 ),
    .funct7( funct7 ),
    .ir_ret( ir_ret ),

    //==ALU Control outputs
    .ALU_A_sel( ALU_A_sel ),
    .ALU_B_sel( ALU_B_sel ),
    .ALU_func( ALU_func ),
    .ALU_func_mod( ALU_mod ),

    //==CompUnit Control output regs
    .CompUnit_cond( CompUnit_cond ),

    //==RegFile Control output regs
    .RegFile_wb_en( RegFile_wb_en ),
    .RegFile_wb_mode( RegFile_wb_mode ),
    .RegFile_rd_sel( RegFile_rd_sel ),

    //==Control information
    .mem_inst( mem_inst ),

    //==Mem write enable  
    .Mem_we( Mem_we ),
    
    .clk( clk ),
    .nreset( nreset )
);

endmodule
`timescale 1ns/1ps
`define GENBUS_WRAPPER
/**********************************************************
Uart Core deisgn
**********************************************************/
module UartCore (
    output reg tx,
    input tx_start,
    output reg tx_done,
    input [7:0] tx_data,

    input rx,
    output reg rx_done,
    output reg [7:0] rx_data,
    input rx_data_unread,

    input en,
    input two_sbit,
    input [1:0] pconf,
    input [15:0] presc,

    //==Flags
    output reg tx_flag,
    input clear_tx_flag,
    output reg rx_flag,
    input clear_rx_flag,
    output reg perr_flag,
    input clear_perr_flag,
    output reg oerr_flag,
    input clear_oerr_flag,

    input clk,
    input nreset
);

localparam EVEN_PARITY = 2'b01;
localparam ODD_PAIRTY = 2'b10;

function get_parity;
input [7:0] data; 
input [1:0] conf;
begin
    case( conf )
        EVEN_PARITY: get_parity = ^data;
        ODD_PAIRTY: get_parity = ~(^data);
        default: get_parity = 1;
    endcase
end
endfunction

reg [3:0] bitcnt_top;
wire use_parity = (pconf == 2'b01) || (pconf == 2'b10);

reg [10:0] tx_frame;
reg [15:0] tx_presc_r;
reg [3:0] tx_bitcnt;
wire tx_pairty;

reg [1:0] rx_sync;
reg rx_on;
reg [10:0] rx_frame;
reg [15:0] rx_presc_r;
reg [3:0] rx_bitcnt;
wire [3:0] rx_bitcnt_top = use_parity ? 9 : 8;

always @*
begin
    case( {^pconf, two_sbit} )
        2'b00: bitcnt_top = 9;
        2'b01: bitcnt_top = 10;
        2'b10: bitcnt_top = 10;
        2'b11: bitcnt_top = 11;
    endcase
end


always @ (posedge clk, negedge nreset )
begin
    if( !nreset ) begin
        tx <= 1;
        tx_done <= 1;
        tx_presc_r <= 0;
        tx_bitcnt <= 0;
    end
    else begin
        if( !en ) begin
            tx <= 1;
            tx_done <= 1;
            tx_presc_r <= 0;
            tx_bitcnt <= 0;
        end
        else if( !tx_done ) begin
            if( tx_presc_r == 0 ) begin
                tx_presc_r <= presc;
                if( tx_bitcnt == bitcnt_top ) begin
                    tx <= 1;
                    tx_done <= 1;
                end
                else begin
                    tx <= tx_frame[ tx_bitcnt ];
                    tx_bitcnt <= tx_bitcnt +1;
                end
            end
            else tx_presc_r <= tx_presc_r -1;
        end
        else begin
            if( tx_start ) begin
                tx <= 0;
                tx_done <= 0;
                tx_bitcnt <= 0;
                tx_presc_r <= presc;
                tx_frame <= {2'b11, get_parity(tx_data, pconf), tx_data};
            end
            else begin
                tx <= 1;
                tx_done <= 1;
                tx_presc_r <= 0;
                tx_bitcnt <= 0;
            end
        end
    end
end

always @ (posedge clk, negedge nreset)
begin
	if( !nreset ) rx_sync <= 2'b11;
	else rx_sync <= {rx_sync[0], rx};
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        rx_frame <= 0;
        rx_data <= 0;
        rx_done <= 0;
        rx_presc_r <= 0;
        rx_bitcnt <= 0;
        rx_on <= 0;
    end
    else begin
        if( !en ) begin
            rx_frame <= 0;
            rx_data <= 0;
            rx_done <= 0;
            rx_presc_r <= 0;
            rx_bitcnt <= 0;
            rx_on <= 0;
        end
        else if( rx_done ) begin
            rx_done <= 0;
        end
        else if( rx_on ) begin
            if( rx_presc_r == 0 ) begin
                rx_presc_r <= presc;
                if( rx_bitcnt == rx_bitcnt_top ) begin
                    rx_on <= 0;
                    rx_done <= 1;
                    if( !rx_data_unread ) rx_data <= rx_frame[7:0];    
                end
                else begin
                    //rx_frame[rx_bitcnt] <= rx;
                    rx_frame[rx_bitcnt] <= rx_sync[1];
                    rx_bitcnt <= rx_bitcnt +1;
                end
            end
            else rx_presc_r <= rx_presc_r -1;
        end
        else begin
            if( !rx_sync[1] ) begin
                rx_on <= 1;
                rx_done <= 0;
                rx_frame <= 11'b11000000000;
                rx_bitcnt <= 0;
                rx_presc_r <= presc + {1'b0, presc[15:1]} -1;
            end
            else begin
                rx_on <= 0;
                rx_done <= 0;
            end
        end
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        tx_flag <= 0;
    end
    else begin
        `ifdef SIMULATION
            
        `endif
        if( tx_flag ) begin
            tx_flag <= clear_tx_flag ? 0 : 1;
        end
        else if( !tx_done && tx_bitcnt == bitcnt_top && tx_presc_r == 0) begin
            tx_flag <= 1;
        end
    end
end

wire rx_finish = rx_on && rx_bitcnt == rx_bitcnt_top && rx_presc_r == 0;

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        rx_flag <= 0;
    end
    else begin
        `ifdef SIMULATION
            
        `endif
        if( rx_flag ) begin
            rx_flag <= clear_rx_flag ? 0 : 1;
        end
        else if( rx_finish ) begin
            rx_flag <= 1'b1;
        end
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        oerr_flag <= 0;
    end
    else begin
        `ifdef SIMULATION
            
        `endif
        if( oerr_flag ) begin
            oerr_flag <= clear_oerr_flag ? 0 : 1;
        end
        else if( rx_finish ) begin
            oerr_flag <= rx_data_unread ? 1 : 0;
        end
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        perr_flag <= 0;
    end
    else begin
        `ifdef SIMULATION
            
        `endif
        if( perr_flag ) begin
            perr_flag <= clear_perr_flag ? 0 : 1;
        end
        else if( rx_finish ) begin
            perr_flag <= use_parity ?
                (rx_frame[8] == get_parity(rx_frame[7:0], pconf) ? 0 : 1) : 0;
        end
    end
end

endmodule

/**********************************************************
Uart Wrapper for generic Bus
The UART Controller has 4 Registers with the following addresses

Addr    Reg             Function
0       Control         Bits[0] -> Enable
                        Bits[1] -> 2 Stopp Bit
                        Bits[3:2] -> 2'b00 : No Parity
                                     2'b01 : Even Parity
                                     2'b10 : Odd Parity
                                     2'b11 : reserved
                        Bits[31:16] -> Prescaler

4       Flag            Bits[0] -> Tx Done Flag
                        Bits[1] -> Rx Done Flag
                        Bits[2] -> Overrun Flag
                        Bits[3] -> Parity Error Flag

NOTE: Flags are cleared by writing 1 to them

8       Write Data      Bits[7:0] -> Data to write
                        Bits[16]   -> TX Done

C       Read Data       Bits[7:0] -> Received DAta
                        Bits[16]  -> RX Done

**********************************************************/
module Uart_GenBusWrapper(
    //==GenBus Interface
    input [31:0] Addr,
    input valid,
    output reg ready,
    input [3:0] we,
    output reg [31:0] Din,
    input [31:0] Dout,

    //==Connection to Uart Core
    output reg tx_start,
    input tx_done,
    output reg [7:0] tx_data,
    output reg rx_data_valid,
    input rx_done,
    input [7:0] rx_data,
    output reg en,
    output reg two_sbit,
    output reg [1:0] pconf,
    output reg [15:0] presc,

    //==Flags
    input tx_flag,
    output clear_tx_flag,
    input rx_flag,
    output clear_rx_flag,
    input perr_flag,
    output clear_perr_flag,
    input oerr_flag,
    output clear_oerr_flag,

    //==Sync Signals
    input clk,
    input nreset
);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Bus Interface

reg [3:0] flag_clear;
wire [31:0] rregs [0:3];
assign rregs[0] = {presc, 12'h0, pconf, two_sbit, en};
assign rregs[1] = {28'h0, perr_flag, oerr_flag, rx_flag, tx_flag};
assign rregs[2] = {15'h0, tx_done, 8'h0, tx_data};
assign rregs[3] = {15'h0, rx_data_valid, 8'h0, rx_data};

wire valid_addr = {Addr[31:4], Addr[1:0]} == 0;
wire waccess = |we;
wire raccess = ~waccess;


//Generate ready pulse
always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        ready <= 0;
    end
    else begin 
        `ifdef SIMULATION
         
        `endif
        //FIXME
        ready <= valid;
    end
end

//Read Process
always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        Din <= 0;
    end
    else begin
        if( valid && !ready && valid_addr && raccess ) begin
            Din <= rregs[ Addr[3:2] ];
        end
    end
end

//Write Process
always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        tx_data <= 0;
        en <= 0;
        two_sbit <= 0;
        pconf <= 2'b00;
        presc <= 16'h4;
    end
    else begin
        if( valid && valid_addr && waccess ) begin
            case( Addr[3:2] )
                2'b00: begin
                    if( en ) begin
                        //Configuration must not be changed
                        en <= Dout[0];
                    end
                    else begin
                        case( we )
                            4'b0001: begin
                                {pconf, two_sbit, en} <= Dout[3:0];
                            end
                            4'b0011: begin
                                {pconf, two_sbit, en} <= Dout[3:0];
                            end
                            4'b1111: begin
                                {pconf, two_sbit, en} <= Dout[3:0];
                                presc <= Dout[31:16];
                            end
                            default: begin
                                en <= en;
                                two_sbit <= two_sbit;
                                pconf <= pconf;
                                presc <= presc;
                            end
                        endcase
                    end
                end
                2'b10: begin
                    tx_data <= tx_done ? Dout[7:0] : tx_data;
                end
                default: begin
                    en <= en;
                    two_sbit <= two_sbit;
                    pconf <= pconf;
                    presc <= presc;
                    tx_data <= tx_data;
                end
            endcase
        end
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        flag_clear <= 0;
    end
    else begin
        if( flag_clear != 0 ) begin
            flag_clear <= 0;
        end
        else begin
            if( valid && !ready && Addr[3:2] == 2'b01 ) begin
                flag_clear <= Dout[3:0];
            end
        end
    end
end
assign clear_tx_flag = flag_clear[0];
assign clear_rx_flag = flag_clear[1];
assign clear_oerr_flag = flag_clear[2];
assign clear_perr_flag = flag_clear[3];

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        tx_start <= 0;
    end
    else begin
        if( tx_start ) begin
            tx_start <= !tx_done ? 0 : 1;
        end
        else if( valid && ready && waccess && Addr[3:2] == 2'b10 ) begin
            tx_start <= tx_done && en ? 1 : 0;
        end
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        rx_data_valid <= 0;
    end
    else begin
        if( rx_data_valid ) begin
            rx_data_valid <= 
                valid && !ready && raccess && Addr[3:2] == 2'b11 ? rx_done : 1;
        end
        else if( rx_done ) rx_data_valid <= 1;
    end
end

endmodule


module Uart
`ifdef CONF_FIX
#(
    parameter TWO_SBIT = 1'b0,
    parameter PCONF = 2'b00,
    parameter PRESC = 16'h0
)
`endif
(
    //==Uart Signals
    input rx,
    output tx,
    output tx_ir_flag,
    output rx_ir_flag,
    output oerr_ir_flag,
    output perr_ir_flag,


`ifdef GENBUS_WRAPPER
    input [31:0] Addr,
    input valid,
    output ready,
    input [3:0] we,
    output [31:0] Din,
    input [31:0] Dout,
`endif

`ifdef NO_WRAPPER
    input tx_start,
    output tx_done,
    input [7:0] tx_data,

    output rx_done,
    output [7:0] rx_data,
    input rx_read_data,
    output rx_perr,
    output rx_ferr,
    output rx_oerr,

    input read_status,
    input en,
    input two_sbit,
    input [1:0] pconf,
    input [15:0] presc,
`endif

    //==Sync Signals
    input clk,
    input nreset
);
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Connection Signals

`ifndef NO_WRAPPER

//==Transmitter
wire [7:0] tx_data;
//==Receiver
wire [7:0] rx_data;
//==Control
wire en;
wire tx_start;
wire rx_data_valid;
wire read_status;
//==Stauts
wire tx_done;
wire rx_done;
wire rx_perr;
wire rx_ferr;
wire rx_oerr;
//==Config
wire two_sbit;
wire [1:0] pconf;
wire [15:0] presc;
//==Flags
wire tx_flag;
wire clear_tx_flag;
wire rx_flag;
wire clear_rx_flag;
wire perr_flag;
wire clear_perr_flag;
wire oerr_flag;
wire clear_oerr_flag;

`endif

assign tx_ir_flag = tx_flag;
assign rx_ir_flag = rx_flag;
assign oerr_ir_flag = oerr_flag;
assign perr_ir_flag = perr_flag;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Instances

UartCore core(
    .tx( tx ),
    .tx_start( tx_start ),
    .tx_done( tx_done ),
    .tx_data( tx_data ),

    .rx( rx ),
    .rx_done( rx_done ),
    .rx_data( rx_data ),
    .rx_data_unread( rx_data_valid ),

    .en( en ),
    .tx_flag( tx_flag ),
    .clear_tx_flag( clear_tx_flag ),
    .rx_flag( rx_flag ),
    .clear_rx_flag( clear_rx_flag ),
    .perr_flag( perr_flag ),
    .clear_perr_flag( clear_perr_flag ),
    .oerr_flag( oerr_flag ),
    .clear_oerr_flag( clear_oerr_flag ),

`ifdef CONF_FIX
    .two_sbit( TWO_SBIT ),
    .pconf( PCONF ),
    .presc( PRESC ),
`else
    .two_sbit( two_sbit ),
    .pconf( pconf ),
    .presc( presc ),
`endif

    .clk( clk ),
    .nreset( nreset )
);

`ifdef GENBUS_WRAPPER
Uart_GenBusWrapper wrapper(
    //==GenBus Interface
    .Addr( Addr ),
    .valid( valid ),
    .ready( ready ),
    .we( we ),
    .Din( Din ),
    .Dout( Dout ),

    //==Connection to Uart Core
    .tx_start( tx_start ),
    .tx_done( tx_done ),
    .tx_data( tx_data ),
    .rx_data_valid( rx_data_valid ),
    .rx_done( rx_done ),
    .rx_data( rx_data ),
    .en( en ),
    .tx_flag( tx_flag ),
    .clear_tx_flag( clear_tx_flag ),
    .rx_flag( rx_flag ),
    .clear_rx_flag( clear_rx_flag ),
    .perr_flag( perr_flag ),
    .clear_perr_flag( clear_perr_flag ),
    .oerr_flag( oerr_flag ),
    .clear_oerr_flag( clear_oerr_flag ),

    .two_sbit( two_sbit ),
    .pconf( pconf ),
    .presc( presc ),

    //==Sync Signals
    .clk( clk ),
    .nreset( nreset )
);
`endif

endmodule

/*******************************************************
Simple GPIO Implementation

Usage:
Addr    0 in
Addr    4 out
Addr    8 individual interrupt enable
Addr    C interrupt flag register
          Flags get deleted by writing 1 to them
Addr   10 Pin state before change
Addr   14 noise filter enable default = 0
Addr   18 noise filter threshold default = 0
*******************************************************/

`timescale 1ns/1ps

module GpioIrUnit #
(
    parameter NUM_CHNL = 8
)
(
    //--Input channels
    input [NUM_CHNL-1:0] gpio_in,

    //--Interrupt Interface
    //----Enables the interrupt for each channel
    input [NUM_CHNL-1:0] ir_en,
    //----Indicates that an ir accured
    output reg [NUM_CHNL-1:0] ir_flag,
    //----Request the clear of a flag
    input [NUM_CHNL-1:0] ir_clear,
    //----Shows the old state before the interrupt
    //----for each chnl independently
    output reg [NUM_CHNL-1:0] old_pin_state,
    //----Request ir handle
    output ir_request,

    //--Sync Signals
    input clk,
    input nreset
);


//=================================================================
//Old State capture
integer chnl;

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        for( chnl=0; chnl<NUM_CHNL; chnl=chnl+1 ) begin
            ir_flag <= 0;
            old_pin_state[chnl] <= 0;
        end
    end
    else begin
        for( chnl=0; chnl<NUM_CHNL; chnl=chnl+1 ) begin
            if( ir_flag[chnl] ) begin
                if( ir_clear[chnl] ) begin
                    ir_flag <= 0;
                    old_pin_state[chnl] <= gpio_in[chnl];
                end
            end
            else begin
                if( gpio_in[chnl] != old_pin_state[chnl] ) begin
                    ir_flag[chnl] <= 1;
                end
                else begin
                    old_pin_state[chnl] <= gpio_in[chnl];
                end
            end
        end
    end
end

assign ir_request = |(ir_en & ir_flag);

endmodule

module Gpio # 
(
    parameter NUM_CHNL = 8
)
(
    //==System Bus
    input [31:0] Addr,
    input valid,
    output reg ready,
    input [3:0] we,
    output reg [31:0] Din,
    input [31:0] Dout,

    //==GPIO Interface
    output reg [NUM_CHNL-1:0] gpio_out,
    input [NUM_CHNL-1:0] gpio_in,

    //==Interrupt
    output ir_request,

    //==Sync
    input clk,
    input nreset
);
//==Write Register Task
function [NUM_CHNL-1:0] write_reg;
input [3:0] we;
input [NUM_CHNL-1:0] dest;
input [31:0] src;
reg [31:0] dest_ext;
reg [31:0] write_reg_ext;
begin
    dest_ext = {{(32-NUM_CHNL){1'b0}}, dest};
    write_reg_ext = we == 4'b0001 ? {dest_ext[31:8], src[7:0]} :
             we == 4'b0011 ? {dest_ext[31:16], src[15:0]} :
             we == 4'b1111 ? src :
             dest_ext;
    write_reg = write_reg_ext[NUM_CHNL-1:0];
end
endfunction

//=================================================================
//Internal Signals
reg [NUM_CHNL-1:0] ir_en;
reg [NUM_CHNL-1:0] ir_clear;
wire [NUM_CHNL-1:0] ir_flag;
wire [NUM_CHNL-1:0] old_pin_state;

wire waccess;
wire [2:0] raddr;

//=================================================================
//Noise Filter Interface
integer fchnl;
reg [7:0] filter_cnt [0:NUM_CHNL-1];
reg [NUM_CHNL-1:0] gpio_in_bit;
reg [7:0] nf_en;
reg [7:0] nf_thresh;

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        for( fchnl=0; fchnl < NUM_CHNL; fchnl=fchnl+1 ) begin
            filter_cnt[fchnl] <= 0;
            gpio_in_bit[fchnl] <= 0;
        end
    end
    else begin
        for( fchnl=0; fchnl < NUM_CHNL; fchnl=fchnl+1 ) begin
            if( nf_en[fchnl] ) begin
                if( filter_cnt[fchnl] == 0 ) begin
                    if( gpio_in[fchnl] != gpio_in_bit[fchnl] ) begin
                        filter_cnt[fchnl] <= nf_thresh;
                    end
                end
                else begin
                    if( gpio_in[fchnl] != gpio_in_bit[fchnl] ) begin
                        filter_cnt[fchnl] <= filter_cnt[fchnl] -1;
                        if( filter_cnt[fchnl] == 1 ) begin
                            gpio_in_bit[fchnl] <= gpio_in[fchnl];
                        end
                    end
                    else begin
                        filter_cnt[fchnl] <= nf_thresh;
                    end
                end
            end
            else begin
                gpio_in_bit[fchnl] <= gpio_in[fchnl];
            end
        end
    end
end

//=================================================================
//Bus Interface
assign waccess = |we;
assign raddr = Addr[4:2];

//=================================================================
//Ready Generator
//  Readys get always generated to not freez the processor
always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        ready <= 0;
    end
    else begin
        `ifdef SIMULATION
            
        `endif
        ready <= valid ? 1 : 0;
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        gpio_out <= 0;
    end
    else if( valid && waccess && raddr == 3'b001 ) begin
        `ifdef SIMULATION
            
        `endif
        gpio_out <= write_reg( we, gpio_out, Dout ); 
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        ir_en <= 0;
    end
    else if( valid && waccess && raddr == 3'b010 ) begin
        `ifdef SIMULATION
            
        `endif
        ir_en <= write_reg( we, ir_en, Dout ); 
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        ir_clear <= 0;
    end
    else begin
        `ifdef SIMULATION
            
        `endif
        if( valid && waccess && raddr == 3'b011 ) begin
            ir_clear <= write_reg( we, ir_clear, Dout ); 
        end
        else begin
            ir_clear <= 0;
        end
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        nf_en <= 0;
    end
    else if( valid && waccess && raddr == 3'b101 ) begin
        `ifdef SIMULATION
            
        `endif
        nf_en <= Dout[7:0];
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        nf_thresh <= 0;
    end
    else if( valid && waccess && raddr == 3'b110 ) begin
        `ifdef SIMULATION
            
        `endif
        nf_thresh <= Dout[7:0];
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        Din <= 32'h0;
    end
    else begin
        Din[31:NUM_CHNL] <= 0;
        if( valid && !ready && !waccess ) begin
            case( raddr )
                3'b000:
                    Din[NUM_CHNL-1:0] <= gpio_in_bit;
                3'b001:
                    Din[NUM_CHNL-1:0] <= gpio_out;
                3'b010:
                    Din[NUM_CHNL-1:0] <= ir_en;
                3'b011:
                    Din[NUM_CHNL-1:0] <= ir_flag;
                3'b100:
                    Din[NUM_CHNL-1:0] <= old_pin_state;
                default:
                    Din[NUM_CHNL-1:0] <= 0;
            endcase
        end
    end
end

GpioIrUnit #
(
    .NUM_CHNL( 8 )
)
gpioIrUnit
(
    //--Input channels
    .gpio_in( gpio_in_bit ),

    //--Interrupt Interface
    //----Enables the interrupt for each channel
    .ir_en( ir_en ),
    //----Indicates that an ir accured
    .ir_flag( ir_flag ),
    //----Request the clear of a flag
    .ir_clear( ir_clear ),
    //----Shows the old state before the interrupt
    //----for each chnl independently
    .old_pin_state( old_pin_state ),
    //----Request ir handle
    .ir_request( ir_request ),

    //--Sync Signals
    .clk( clk ),
    .nreset( nreset )
);

endmodule

/* verilator lint_off WIDTH */
/* NOTE_1:
   Potentially dangerous! At Note_2 more index bits
   than available might be used. This is catched
   here by the if before the access that checks if
   the address out of bound. By setting the 
   Verilator Command above width checks are turned off.
   This is necessary but should only be done in the
   final round to see other width errors  !!!!
*/
module Irc (
    //==Interrupt Interface
    input [4:0] ir_channel,

    //==Interrupt Core Interface
    output reg ir_request,
    input ir_grant,
    output reg [31:0] ivec,

    //==System Bus Interface
    input [31:0] Addr,
    input valid,
    output reg ready,
    input [3:0] we,
    input [31:0] Dout,
    output reg [31:0] Din,

    //==Sync
    input clk,
    input nreset
);

localparam NUM_IR_CHANNEL = 5;
//==Write Register Task
function [31:0] write_register;
input [3:0] we;
input [31:0] dest;
input [31:0] src;
begin
    write_register = we == 4'b0001 ? {dest[31:8], src[7:0]} :
             we == 4'b0011 ? {dest[31:16], src[15:0]} :
             we == 4'b1111 ? src :
             dest;
end
endfunction

//==Interrupt Vector Registers
wire [NUM_IR_CHANNEL-1:0] ir_mask;
wire [31:0] ir_vectors [0:NUM_IR_CHANNEL-1];
reg ir_processing;
integer i;
integer k;

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        ir_request <= 0;
        ir_processing <= 0;
        ivec <= 32'h0;
    end
    else begin
        if( ir_processing ) begin
            if( !ir_grant ) begin
                ir_processing <= 0;
            end
        end
        else if( ir_request ) begin
            if( ir_grant ) begin
                ir_request <= 0;
                ir_processing <= 1;
            end
        end
        else begin
            ir_request <= 0;
            ivec <= 0;
            for( i=NUM_IR_CHANNEL-1; i >= 0; i = i-1 ) begin: IrCompGen
                if( ir_channel[i] && ir_mask[i] ) begin
                    ir_request <= 1;
                    ivec <= ir_vectors[i];
                end
            end
        end
    end
end

//==Bus Interface
wire waccess = |we;
reg [31:0] bus_register [0:NUM_IR_CHANNEL];

assign ir_mask = bus_register[0][NUM_IR_CHANNEL-1:0];

generate
genvar j;
    for( j=0; j<NUM_IR_CHANNEL; j = j+1 ) begin : GEN_ASSIGN
        assign ir_vectors[j] = bus_register[j+1];
    end
endgenerate 

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        ready <= 0;
        Din <= 0;
        bus_register[0] <= 32'h0;
        bus_register[1] <= 32'h0;
        bus_register[2] <= 32'h0;
        bus_register[3] <= 32'h0;
        bus_register[4] <= 32'h0;
        bus_register[5] <= 32'h0;
    end
    else begin
        `ifdef SIMULATION
            
        `endif
        if( ready ) begin
            ready <= valid ? 1 : 0;
        end
        else if( valid ) begin
            ready <= 1;
            /* Note_2
               Addr is first checkt!
            */
            if( Addr[7:2] < NUM_IR_CHANNEL+1 ) begin
                if( waccess ) begin
                    bus_register[Addr[7:2]] <=
                        write_register( we, bus_register[Addr[7:2]], Dout ); 
                end
                else begin
                    Din <= bus_register[Addr[7:2]];
                end
            end
        end
    end
end

endmodule
/*********************************************************************
    Register Space
    0x00 :  Bit[0] -> en
            Bit[1] -> cpol
            Bit[2] -> cpha
            Bit[12:8] -> trans len
            Bit[31:16] -> prescaler

    0x04:   Bit[0] -> done_flag
            The flag is cleared by either write to Data Reg or
            writing 1 to flag reg
            Bit[1] -> busy
    
    0x08 :  Data Reg

*********************************************************************/

module Spi(
    //Spi Bus
    output sck,
    output reg mosi,
    input miso,

    //==System Bus Interface
    input [31:0] Addr,
    input valid,
    output reg ready,
    input [3:0] we,
    input [31:0] Dout,
    output reg [31:0] Din,

    output ir_request,

    //==Sync Signals
    input clk,
    input nreset
);

//==Write Register Task
function [31:0] write_reg;
input [3:0] we;
input [31:0] dest;
input [31:0] src;
begin
    write_reg= we == 4'b0001 ? {dest[31:8], src[7:0]} :
             we == 4'b0011 ? {dest[31:16], src[15:0]} :
             we == 4'b1111 ? src :
             dest;
end
endfunction

reg [31:0] cfg_reg;
reg clear_flag;
reg new_data;
wire waccess = |we;
wire addr_valid = (Addr[1:0] == 2'b00) && (Addr < 32'hC);

reg done_flag;
reg trans_on;
wire en = cfg_reg[0];
wire cpol = cfg_reg[1];
wire cpha = cfg_reg[2];
wire [4:0] trans_len = cfg_reg[12:8];
wire [15:0] presc_ratio = cfg_reg[31:16];

reg trans_start;
reg [31:0] wdata;
reg [31:0] rdata;


/*******************************************************************
Integrated Submodule: Prescaler
*******************************************************************/
reg [15:0] presc_r;
reg [4:0] bit_counter;
reg spi_clk;
reg spi_clk_ff;
wire spi_clk_posedge = spi_clk & ~spi_clk_ff;
wire spi_clk_negedge = ~spi_clk & spi_clk_ff;

/***************************************************************************
    Bus Functionality
***************************************************************************/

//==Ready gen process
always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        ready <= 0;
    end
    else begin
        /*
        `ifdef SIMULATION
            
        `endif
        */
        ready <= valid;
    end
end

//==Read Process
always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        Din <= 32'h0;
    end
    else begin
        if( !valid || !addr_valid ) begin
            Din <= 32'h0;
        end
        else if( valid && !ready && !waccess ) begin
            case( Addr[3:2] )
                2'b00:
                    Din <= cfg_reg;
                2'b01:
                    Din <= {30'h0, trans_on, done_flag};
                2'b10:
                    Din <= trans_on ? 32'h0 : rdata;
                default:
                    Din <= 32'h0;
            endcase
        end
    end
end

//==Write Processes
always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        cfg_reg <= 32'h0;
    end
    else begin
        /*
        `ifdef SIMULATION
            
        `endif
        */
        if( Addr[3:2] == 2'b00 && waccess && valid && !ready ) begin
            cfg_reg <= write_reg( we, cfg_reg, Dout );
        end
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        clear_flag <= 1'b0;
    end
    else begin
        /*
        `ifdef SIMULATION
            
        `endif
        */
        if( clear_flag ) begin
            clear_flag <= 1'b0;
        end
        else if( Addr[3:2] == 2'b01 && waccess && valid && !ready ) begin
            clear_flag <= Dout[0];
        end
    end
end

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        new_data <= 1'b0;
        wdata <= 32'h0;
    end
    else begin
        /*
        `ifdef SIMULATION
            
        `endif
        */
        if( new_data ) begin
            new_data <= 1'b0;
        end
        else if( !trans_on ) begin
            if( Addr[3:2] == 2'b10 && waccess && valid && !ready ) begin
                new_data <= 1'b1;
                wdata <= write_reg( we, 32'h0, Dout );
            end
        end
    end
end



/***************************************************************************
    Core Functionality
***************************************************************************/

always @(posedge clk, negedge nreset )
begin
    if( !nreset ) begin
        presc_r <= 0;
        spi_clk <= 0;
        spi_clk_ff <= 0;
    end
    else begin
        
        if( en ) begin
            if( trans_on || presc_r != 0) begin
                presc_r <= presc_r == 0 ? (presc_ratio>>1)-1 : presc_r-1;
                spi_clk <= presc_r == 0 ? ~spi_clk : spi_clk;
                spi_clk_ff <= spi_clk;
            end
            else begin
                presc_r <= {presc_ratio[15:2],1'b0};
                spi_clk <= cpol;
                spi_clk_ff <= cpol;
            end
        end
        else begin
            presc_r <= 16'h0;
            spi_clk <= 1'b0;
            spi_clk_ff <= 1'b0;
        end
    end
end

//////////////////////////////////////////////////////////////////////////

wire spi_read = cpol ^ cpha ? spi_clk_negedge : spi_clk_posedge;
wire spi_write = cpol ^ cpha ? spi_clk_posedge : spi_clk_negedge;

always @(posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        trans_on <= 0;
        bit_counter <= 0;
        mosi <= 0;
        rdata <= 32'h0;
    end
    else begin
        if( !en ) begin
            trans_on <= 1'b0;
        end
        else begin
            if( trans_on ) begin
                if( spi_write  ) begin
                    mosi <= wdata[ bit_counter ];
                end
                else if( spi_read  ) begin
                    rdata[ bit_counter ] <= miso;
                    if( bit_counter == 0 ) begin
                        trans_on <= 0;
                        bit_counter <= 0;
                    end 
                    else begin
                        trans_on <= 1;
                        bit_counter <= bit_counter -1;
                    end
                end
            end
            else begin
                if( new_data ) begin  
                    trans_on <= 1;
                    bit_counter <= trans_len-1;
                    mosi <= wdata[ trans_len-1 ];
                end
            end
        end
    end
end

always @ (posedge clk, negedge nreset )
begin
    if( !nreset ) begin
        done_flag <= 1'b0;
    end
    else begin
        if( !en ) begin
            done_flag <= 1'b0;
        end
        else if( done_flag ) begin
            if( clear_flag || new_data ) done_flag <= 1'b0;
        end
        else begin
            if( trans_on && spi_read && bit_counter == 0 ) done_flag <= 1'b1;
        end
    end
end

assign sck = spi_clk;
assign ir_request = done_flag;

endmodule

`timescale 1ns/1ps
module FlashSpi(
    //==Spi Interface
    output reg mosi,
    input miso,
    output reg sck,

    //==Control
    input write_byte,
    input write_word,

    output reg done,
    input [31:0] wdata,
    output reg [31:0] rdata, 

    //==Config
    input [3:0] presc,

    //==Sync Signals
    input clk,
    input nreset
);

//Start Detection 
wire trans_req;
reg trans_start_ff;
wire trans_start;

reg [3:0] presc_r;

reg [4:0] bitcnt;
wire [4:0] bitcnt_max;

reg sck_ff;
wire sck_posedge;
wire sck_negedge;

//==Start Detector
//..The Master can request a Transfer by setting one of the
//..bove write signals to one. Once set, the write signal must
//..remain high until the end of the transfer.
//
//..After accepting the request, the done flag goes to zero.
//
//..As soon as the transfer is finished, the done flag goes to one and
//..remains in this state until the next start

assign trans_req = write_byte | write_word;

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        trans_start_ff <= 0;
    end
    else begin
        trans_start_ff <= trans_req;
    end
end

assign trans_start = trans_req & ~trans_start_ff;

//==Spi Baud/Clock generator
//..The Clock generated produces the prescaled clock for the spi
//..interface. For this module only Spi Mode 0 is supported.
//..Hence the clock is idle 0
//..If no transfer is on going the clock line remain 0

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        sck <= 1'b0;
        presc_r <= 0;
    end
    else begin
        if( done ) begin
            //presc_r <= 4'b0;
            //presc_r <= (presc -1) >>1;
            presc_r <= 4'b0001;
        end
        else begin
            if( presc_r == 4'b0 ) begin
                sck <= ~sck;
                //presc_r <= done ? 4'b0 : (presc -1) >>1;
                presc_r <= 4'b0001;
            end
            else begin
                presc_r <= presc_r -1;
            end
        end
    end
end

//==Sck Edge Detector
//..For the communication it is important that the spi
//..circuit works on the edges of the sck signal.
//..For this purpose a edge detecter is build

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        sck_ff <= 1'b0;
    end
    else begin
        sck_ff <= sck;
    end
end

assign sck_posedge = ~sck_ff & sck;
assign sck_negedge = sck_ff & ~sck;

//==Spi Communication
//..This process realizes the main spi communication.
//..After receiving the start request, the circuit starts clocking
//..out the write data on the positive edge. On the negative edge
//..the incomming data is sampled. After finishing the 
//..transfer, the done flag is asserted

assign bitcnt_max = write_byte ? 5'd7 : 
                    write_word ? 5'd31 :
                    5'd0;

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        bitcnt <= 5'b0;
        mosi <= 1'b0;
        done <= 1'b1;
    end
    else begin
        if( !done ) begin
            if( sck_posedge ) begin
                rdata[bitcnt] <= miso;
            end
            else if( sck_negedge ) begin
                mosi <= bitcnt == 5'b0 ? 1'b0 : wdata[bitcnt-1];
                bitcnt <= bitcnt -1;
                if( bitcnt == 5'b0 ) begin
                    done <= 1'b1;
                end
            end
        end
        else begin
            mosi <= 1'b0;
            if( trans_start ) begin
                done <= 1'b0;
                bitcnt <= bitcnt_max;
                mosi <= wdata[bitcnt_max];
            end
        end
    end
end

endmodule

module FlashFsm(
    //==Arg Inputs
    input [16:0] FWAddr,
    input [7:0] raw_byte,
    input raw_cs,

    //==Cmd Inputs
    input rom_read,
    input raw_write,
    input raw_change_cs,
    output idle,

    //==FlashSpi Interface
    output [31:0] wdata,
    output reg write_word,
    output reg write_byte,
    input done,

    //==Spi Interface
    output reg cs,

    //==Sync Signals
    input clk,
    input nreset
);

reg page_valid;
reg [16:0] next_FWAddr;

reg [7:0] state;

localparam s_idle           = 8'b00000001;
localparam s_ack_rrom       = 8'b00000010;
localparam s_rrom           = 8'b00000100;
localparam s_start_orom     = 8'b00001000;
localparam s_ack_waddr_rom  = 8'b00010000;
localparam s_waddr_rom      = 8'b00100000;
localparam s_ack_raww       = 8'b01000000;
localparam s_raww           = 8'b10000000;

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        page_valid <= 1'b0;
        next_FWAddr <= 17'b0;
        state <= s_idle;
        cs <= 1'b1;
        write_word <= 1'b0;
        write_byte <= 1'b0;
    end
    else begin
        case( state )
            s_idle: begin
                if( rom_read ) begin
                    if( page_valid && next_FWAddr == FWAddr ) begin
                        write_word <= 1'b1;
                        state <= s_ack_rrom;
                    end
                    else begin
                        cs <= 1'b1;
                        state <= s_start_orom;
                    end
                end
                else if( raw_change_cs ) begin
                    page_valid <= 1'b0;
                    state <= s_ack_raww;
                end
                else if( raw_write ) begin
                    page_valid <= 1'b0;
                    write_byte <= 1'b1;
                    state <= s_ack_raww;
                end
            end

            //==Open new Flash page
            s_start_orom: begin
                cs <= 1'b0;
                write_word <= 1'b1;
                state <= s_ack_waddr_rom;
            end
            s_ack_waddr_rom: begin
                if( !done ) begin
                    state <= s_waddr_rom;
                end
            end
            s_waddr_rom: begin
                if( done ) begin
                    state <= s_ack_rrom;
                    write_word <= 1'b0;
                    page_valid <= 1'b1;
                end
            end

            //==Read one word from word
            s_ack_rrom: begin
                write_word <= 1'b1;
                if( !done ) begin
                    state <= s_rrom;
                end
            end
            s_rrom: begin
                if( done ) begin
                    write_word <= 1'b0;
                    next_FWAddr <= FWAddr + 1;
                    state <= s_idle;
                end
            end
            default:
                state <= s_idle;

            //==Raw Byte/CS Access
            s_ack_raww: begin
                if( raw_change_cs ) begin
                    cs <= raw_cs;
                    state <= s_idle;
                end
                else if( !done ) begin
                    write_byte <= 1'b0;
                    state <= s_raww;
                end
            end
            s_raww: begin
                if( done ) state <= s_idle;
            end

        endcase
    end
end

assign idle = state[0];

assign wdata[7:0] = (state == s_ack_raww || state == s_raww) ? raw_byte :
                    (state == s_ack_waddr_rom || state == s_waddr_rom) ? {FWAddr[5:0],2'b00} :
                    8'h0;

assign wdata[18:8] = (state == s_ack_waddr_rom || state == s_waddr_rom) ? FWAddr[16:6] : 11'b0;

assign wdata[23:19] = 5'b0;
assign wdata[31:24] = (state == s_ack_waddr_rom || state == s_waddr_rom) ? 8'h03 : 8'h0;

endmodule

module FlashCtrl(
    //==Spi Interface
    output mosi,
    input miso,
    output sck,
    output cs,

    //==System Bus Interface
    input [31:0] Addr,
    input valid,
    output reg ready,
    input [3:0] we,
    input [31:0]  Dout,
    output reg [31:0] Din,

    //==Sync Signals
    input clk,
    input nreset
);
//==Global Signals
//..These signals are used at multiple points in the controller

wire [31:0] rdata;
reg busy;
reg ack;
wire idle;
wire waccess = |we;

//==Rom Read
//..In the rom read mode the spi controller act as a normal Rom
//..Component. The Rom mode is very slow compared to a real parallel
//..component but this way memory resources can be shifted of chip
reg rom_read;
wire [16:0] FWAddr;

always @ (posedge clk, negedge nreset)
begin
    if( !nreset )  begin
        rom_read <= 1'b0;
    end
    else begin
        if( rom_read ) begin
            if( !idle ) rom_read <= 1'b0;
        end
        else if( !busy && !ready ) begin
            if( valid && Addr[26:24] == 4'b100 && !waccess) begin
                rom_read <= 1'b1;
            end
        end 
    end
end

assign FWAddr = Addr[18:2];

//==Raw Byte Read/Write
//..During Flash operation it is important to have raw protocoll less
//..access to the flash component. For this scenario the flash controller
//..offers the raw mode. The Raw mode is entered by addressing the
//..Controller a the Address 0x04XXXXXX. For a write access the
//..lower 8 bit of the Dout port are written. For a read a zero byte 
//..is trnasmitted and the answer is presented in Din[7:0]

reg raw_write;
reg [7:0] raw_byte;

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        raw_write <= 1'b0;
    end
    else begin
        if( raw_write ) begin
            if( !idle ) raw_write <= 1'b0;
        end
        else if( !busy && !ready ) begin
            if( valid && Addr[26:24] == 4'b010 ) begin
                raw_write <= 1'b1;
                raw_byte <= waccess ? Dout[7:0] : 8'h0;
            end
        end
    end
end

//==Raw CS access
//..To control the Flash coperations independently it is necessary to
//..be able to change the CS line at will. For this purpose a raw cs
//..mode is offered. If the Flash controller is accessed in a write
//..access to the address 0x02XXXXXX the CS line is set to Dout[0]
reg raw_change_cs;

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        raw_change_cs <= 1'b0;
    end
    else begin
        if( raw_change_cs ) begin
            if( !idle ) raw_change_cs <= 1'b0;
        end
        else if( !busy ) begin
            if( valid && Addr[26:24] == 4'b001 && waccess ) begin
                raw_change_cs <= 1'b1;
            end
        end
    end
end


wire req_issued = rom_read | raw_write | raw_change_cs;

always @ (posedge clk, negedge nreset)
begin
    if( !nreset ) begin
        Din <= 32'h0;
        ready <= 1'b0;
        busy <= 1'b0;
        ack <= 1'b0;
    end
    else begin
        if( ready ) begin
            busy <= 1'b0;
            if( !valid ) ready <= 1'b0;
        end
        else if( busy ) begin
            if( ack ) begin
                if( idle ) begin
                    ack <= 1'b0;
                     ready <= 1'b1;
                    if( !waccess ) 
                        Din <= waccess ? 32'h0 : {rdata[7:0], rdata[15:8], rdata[23:16], rdata[31:24]};
                    else Din <= 32'h0; 
                end
            end
            else begin
                if( !idle ) begin
                    ack <= 1'b1;
                end
                else if( !req_issued ) begin
                    ready <= 1'b1;
                end
            end
        end
        else if( valid ) begin
            busy <= 1'b1;
        end
    end
end

//==Inter instance connection Signals
//..These signals are used to connect functions of the
//..instanciated modules to each other.
wire write_byte;
wire write_word;
wire done;
wire [3:0] presc = 4'b1001;
wire [31:0] wdata;

FlashSpi flashSpi(
    //==Spi Interface
    .mosi( mosi ),
    .miso( miso ),
    .sck( sck ),

    //==Control
    .write_byte( write_byte ),
    .write_word( write_word ),

    .done( done ),
    .wdata( wdata ),
    .rdata( rdata ),

    //==Config
    .presc( presc ),

    //==Sync Signals
    .clk( clk ),
    .nreset( nreset )
);

FlashFsm flashFsm(
    //==Arg Inputs
    .FWAddr( FWAddr ),
    .raw_byte( raw_byte ),
    .raw_cs( Dout[0] ),

    //==Cmd Inputs
    .rom_read( rom_read ),
    .raw_write( raw_write ),
    .raw_change_cs( raw_change_cs ),
    
    //==FlashSpi Interface
    .wdata( wdata ),
    .write_word( write_word ),
    .write_byte( write_byte ),
    .done( done ),
    .idle( idle ),

    //==Spi Interface
    .cs( cs ),

    //==Sync Signals
    .clk( clk ),
    .nreset( nreset )
);

endmodule

module BusInterface # 
(
    parameter GEN_READY = 1'b0,
    parameter PIPELINED = 1'b0,
    parameter ADDR_MASK = 32'hF0000000
)
(
    input en,

    input [31:0] s_Addr,
    input s_valid,
    output s_ready,
    input [3:0] s_we,
    input [31:0] s_Dout,
    output [31:0] s_Din,

    output [31:0] m_Addr,
    output m_valid,
    input m_ready,
    output [3:0] m_we,
    output [31:0] m_Dout,
    input [31:0] m_Din,

    input clk,
    input nreset
);

generate
if( GEN_READY ) begin: ReadyGenProc
    reg s_ready_r;
    always @ (posedge clk, negedge nreset)
    begin
        if( !nreset ) begin
            s_ready_r <= 0;
        end
        else begin
            `ifdef SIMULATION
                
            `endif
            if( s_ready_r ) begin
                s_ready_r <= 0;
            end
            else begin
                s_ready_r <= s_valid ? 1 : 0;
            end
        end
    end
    assign s_ready = en ? s_ready_r : 0;
end
endgenerate

generate
if( PIPELINED ) begin: ReadyPipeProc
    reg s_ready_p;
    always @ (posedge clk, negedge nreset)
    begin
        if( !nreset ) begin
            s_ready_p <= 0;
        end
        else begin
            `ifdef SIMULATION
                
            `endif
            s_ready_p <= m_ready;
        end
    end
    assign s_ready = en ? s_ready_p : 0;
end
endgenerate

generate
if(!GEN_READY  && !PIPELINED) begin: ReadyMux
    assign s_ready = en ? m_ready : 0;
end
endgenerate

generate
if( PIPELINED ) begin: PipSigProc
    reg [31:0] s_Addr_p;
    reg s_valid_p;
    reg [3:0] s_we_p;
    reg [31:0] s_Dout_p;
    reg [31:0] s_Din_p;

    always @ (posedge clk, negedge nreset) begin
        if( !nreset ) begin
            s_Addr_p <= 0;
            s_valid_p <= 0;
            s_we_p <= 0;
            s_Dout_p <= 0;
            s_Din_p <= 0;
        end
        else begin
            `ifdef SIMULATION
                
            `endif
            if( en ) begin
                s_Addr_p <= s_Addr & ~ADDR_MASK;
                s_valid_p <= s_valid;
                s_we_p <= s_we;
                s_Dout_p <= s_Dout;
                s_Din_p <= m_Din;
            end
            else begin
                s_valid_p <= 0;
            end
        end
    end

    assign m_Addr = en ? s_Addr_p : 0;
    assign m_valid = en ? s_valid_p : 0;
    assign m_we = en ? s_we_p : 0;
    assign m_Dout = en ? s_Dout_p : 0;
    assign s_Din = en ? s_Din_p : 0;
end
endgenerate

generate
if( !PIPELINED ) begin: SigMux
    assign m_Addr = en ? s_Addr & ~ADDR_MASK : 0;
    assign m_valid = en ? s_valid : 0;
    assign m_we = en ? s_we : 0;
    assign m_Dout = en ? s_Dout : 0;
    assign s_Din = en ? m_Din : 0;
end
endgenerate

endmodule 

module MemBusSwitch # 
(
    parameter M0_BASE_ADDR = 32'h0000000,
    parameter M0_ADDR_MASK = 32'hf000000,

    parameter M1_BASE_ADDR = 32'h0000000,
    parameter M1_ADDR_MASK = 32'hf000000,

    parameter M2_BASE_ADDR = 32'h0000000,
    parameter M2_ADDR_MASK = 32'hf000000
)
(
    input [31:0] s0_Addr,
    input s0_valid,
    output reg s0_ready,
    input [3:0] s0_we,
    input [31:0] s0_Dout,
    output reg [31:0] s0_Din,

    output [31:0] m0_Addr,
    output m0_valid,
    input m0_ready,
    output [3:0] m0_we,
    output [31:0] m0_Dout,
    input [31:0] m0_Din,

    output [31:0] m1_Addr,
    output m1_valid,
    input m1_ready,
    output [3:0] m1_we,
    output [31:0] m1_Dout,
    input [31:0] m1_Din,

    output [31:0] m2_Addr,
    output m2_valid,
    input m2_ready,
    output [3:0] m2_we,
    output [31:0] m2_Dout,
    input [31:0] m2_Din,

    output [31:0] md_Addr,
    output md_valid,
    input md_ready,
    output [3:0] md_we,
    output [31:0] md_Dout,
    input [31:0] md_Din,

    input clk,
    input nreset
);

//Master 0 Signals
wire s0_ready_ret;
wire [31:0] s0_Din_ret;

wire m0_match = (s0_Addr & M0_ADDR_MASK) == M0_BASE_ADDR;

//Master 1 Signals
wire s1_ready_ret;
wire [31:0] s1_Din_ret;

wire m1_match = (s0_Addr & M1_ADDR_MASK) == M1_BASE_ADDR;

//Master 2 Signals
wire s2_ready_ret;
wire [31:0] s2_Din_ret;

wire m2_match = (s0_Addr & M2_ADDR_MASK) == M2_BASE_ADDR;


//Default Interface
wire sd_ready_ret;
wire [31:0] sd_Din_ret;
wire md_match = !m0_match && !m1_match && !m2_match;

//==Match Vector and Din/ready assignment
wire [2:0] match_vec = {m2_match, m1_match, m0_match};

always @*
begin
    case( match_vec )
        3'b001: begin
            s0_Din = s0_Din_ret;
            s0_ready = s0_ready_ret;
        end
        3'b010: begin
            s0_Din = s1_Din_ret;
            s0_ready = s1_ready_ret;
        end
        3'b100: begin
            s0_Din = s2_Din_ret;
            s0_ready = s2_ready_ret;
        end
        default: begin
            s0_Din = sd_Din_ret;
            s0_ready = sd_ready_ret;
        end
    endcase
end

BusInterface # 
(
    .GEN_READY( 1 ),
    .PIPELINED( 0 ),
    .ADDR_MASK( M0_ADDR_MASK )
)
romInterface
(
    .en( m0_match ),

    .s_Addr( s0_Addr ),
    .s_valid( s0_valid ),
    .s_ready( s0_ready_ret ),
    .s_we( s0_we ),
    .s_Dout( s0_Dout ),
    .s_Din( s0_Din_ret ),

    .m_Addr( m0_Addr ),
    .m_valid( m0_valid ),
    .m_ready( m0_ready ),
    .m_we( m0_we ),
    .m_Dout( m0_Dout ),
    .m_Din( m0_Din ),

    .clk( clk ),
    .nreset( nreset )
);

BusInterface # 
(
    .GEN_READY( 1 ),
    .PIPELINED( 0 ),
    .ADDR_MASK( M1_ADDR_MASK )
)
ramInterface
(
    .en( m1_match ),

    .s_Addr( s0_Addr ),
    .s_valid( s0_valid ),
    .s_ready( s1_ready_ret ),
    .s_we( s0_we ),
    .s_Dout( s0_Dout ),
    .s_Din( s1_Din_ret ),

    .m_Addr( m1_Addr ),
    .m_valid( m1_valid ),
    .m_ready( m1_ready ),
    .m_we( m1_we ),
    .m_Dout( m1_Dout ),
    .m_Din( m1_Din ),

    .clk( clk ),
    .nreset( nreset )
);

BusInterface # 
(
    .GEN_READY( 0 ),
    .PIPELINED( 0 ),
    .ADDR_MASK( M2_ADDR_MASK )
)
spiRomInterface
(
    .en( m2_match ),

    .s_Addr( s0_Addr ),
    .s_valid( s0_valid ),
    .s_ready( s2_ready_ret ),
    .s_we( s0_we ),
    .s_Dout( s0_Dout ),
    .s_Din( s2_Din_ret ),

    .m_Addr( m2_Addr ),
    .m_valid( m2_valid ),
    .m_ready( m2_ready ),
    .m_we( m2_we ),
    .m_Dout( m2_Dout ),
    .m_Din( m2_Din ),

    .clk( clk ),
    .nreset( nreset )
);

BusInterface # 
(
    .GEN_READY( 0 ),
    .PIPELINED( 1 ),
    .ADDR_MASK( 32'h00000000 )
)
defaultInterface
(
    .en( md_match ),

    .s_Addr( s0_Addr ),
    .s_valid( s0_valid ),
    .s_ready( sd_ready_ret ),
    .s_we( s0_we ),
    .s_Dout( s0_Dout ),
    .s_Din( sd_Din_ret ),

    .m_Addr( md_Addr ),
    .m_valid( md_valid ),
    .m_ready( md_ready ),
    .m_we( md_we ),
    .m_Dout( md_Dout ),
    .m_Din( md_Din ),

    .clk( clk ),
    .nreset( nreset )
);

endmodule

module PeripherieBusSwitch # 
(
    parameter M0_BASE_ADDR = 32'h0000000,
    parameter M0_ADDR_MASK = 32'hf000000,

    parameter M1_BASE_ADDR = 32'h0000000,
    parameter M1_ADDR_MASK = 32'hf000000,

    parameter M2_BASE_ADDR = 32'h00000000,
    parameter M2_ADDR_MASK = 32'hf0000000,

    parameter M3_BASE_ADDR = 32'h00000000,
    parameter M3_ADDR_MASK = 32'hf0000000
)
(
    input [31:0] s0_Addr,
    input s0_valid,
    output reg s0_ready,
    input [3:0] s0_we,
    input [31:0] s0_Dout,
    output reg [31:0] s0_Din,

    output [31:0] m0_Addr,
    output m0_valid,
    input m0_ready,
    output [3:0] m0_we,
    output [31:0] m0_Dout,
    input [31:0] m0_Din,

    output [31:0] m1_Addr,
    output m1_valid,
    input m1_ready,
    output [3:0] m1_we,
    output [31:0] m1_Dout,
    input [31:0] m1_Din,

    output [31:0] m2_Addr,
    output m2_valid,
    input m2_ready,
    output [3:0] m2_we,
    output [31:0] m2_Dout,
    input [31:0] m2_Din,

    output [31:0] m3_Addr,
    output m3_valid,
    input m3_ready,
    output [3:0] m3_we,
    output [31:0] m3_Dout,
    input [31:0] m3_Din,

    input clk,
    input nreset
);

//Master 0 Signals
wire s0_ready_ret;
wire [31:0] s0_Din_ret;

wire  m0_match = (s0_Addr & M0_ADDR_MASK) == M0_BASE_ADDR;

//Master 1 Signals
wire s1_ready_ret;
wire [31:0] s1_Din_ret;

wire  m1_match = (s0_Addr & M1_ADDR_MASK) == M1_BASE_ADDR;

//Master 2 Signals
wire s2_ready_ret;
wire [31:0] s2_Din_ret;

wire  m2_match = (s0_Addr & M2_ADDR_MASK) == M2_BASE_ADDR;

//Master 3 Signals
wire s3_ready_ret;
wire [31:0] s3_Din_ret;

wire  m3_match = (s0_Addr & M3_ADDR_MASK) == M3_BASE_ADDR;


//==Match Vector and Din/ready assignment
wire [3:0] match_vec = {m3_match, m2_match, m1_match, m0_match};

always @*
begin
    case( match_vec )
        4'b0001: begin
            s0_Din = s0_Din_ret;
            s0_ready = s0_ready_ret;
        end
        4'b0010: begin
            s0_Din = s1_Din_ret;
            s0_ready = s1_ready_ret;
        end
        4'b0100: begin
            s0_Din = s2_Din_ret;
            s0_ready = s2_ready_ret;
        end
        4'b1000: begin
            s0_Din = s3_Din_ret;
            s0_ready = s3_ready_ret;
        end
        default: begin
            s0_Din = 32'h0;
            s0_ready = 0;
        end
    endcase
end

BusInterface # 
(
    .GEN_READY( 0 ),
    .PIPELINED( 0 ),
    .ADDR_MASK( M0_ADDR_MASK )
)
gpioInterface
(
    .en( m0_match ),

    .s_Addr( s0_Addr ),
    .s_valid( s0_valid ),
    .s_ready( s0_ready_ret ),
    .s_we( s0_we ),
    .s_Dout( s0_Dout ),
    .s_Din( s0_Din_ret ),

    .m_Addr( m0_Addr ),
    .m_valid( m0_valid ),
    .m_ready( m0_ready ),
    .m_we( m0_we ),
    .m_Dout( m0_Dout ),
    .m_Din( m0_Din ),

    .clk( clk ),
    .nreset( nreset )
);

BusInterface # 
(
    .GEN_READY( 0 ),
    .PIPELINED( 0 ),
    .ADDR_MASK( M1_ADDR_MASK )
)
uartInterface
(
    .en( m1_match ),

    .s_Addr( s0_Addr ),
    .s_valid( s0_valid ),
    .s_ready( s1_ready_ret ),
    .s_we( s0_we ),
    .s_Dout( s0_Dout ),
    .s_Din( s1_Din_ret ),

    .m_Addr( m1_Addr ),
    .m_valid( m1_valid ),
    .m_ready( m1_ready ),
    .m_we( m1_we ),
    .m_Dout( m1_Dout ),
    .m_Din( m1_Din ),

    .clk( clk ),
    .nreset( nreset )
);

BusInterface # 
(
    .GEN_READY( 0 ),
    .PIPELINED( 0 ),
    .ADDR_MASK( M2_ADDR_MASK )
)
spiInterface
(
    .en( m2_match ),

    .s_Addr( s0_Addr ),
    .s_valid( s0_valid ),
    .s_ready( s2_ready_ret ),
    .s_we( s0_we ),
    .s_Dout( s0_Dout ),
    .s_Din( s2_Din_ret ),

    .m_Addr( m2_Addr ),
    .m_valid( m2_valid ),
    .m_ready( m2_ready ),
    .m_we( m2_we ),
    .m_Dout( m2_Dout ),
    .m_Din( m2_Din ),

    .clk( clk ),
    .nreset( nreset )
);

BusInterface # 
(
    .GEN_READY( 0 ),
    .PIPELINED( 0 ),
    .ADDR_MASK( M3_ADDR_MASK )
)
ircInterface
(
    .en( m3_match ),

    .s_Addr( s0_Addr ),
    .s_valid( s0_valid ),
    .s_ready( s3_ready_ret ),
    .s_we( s0_we ),
    .s_Dout( s0_Dout ),
    .s_Din( s3_Din_ret ),

    .m_Addr( m3_Addr ),
    .m_valid( m3_valid ),
    .m_ready( m3_ready ),
    .m_we( m3_we ),
    .m_Dout( m3_Dout ),
    .m_Din( m3_Din ),

    .clk( clk ),
    .nreset( nreset )
);

endmodule

module RiscyBoy(
    input [7:0] gpio_in,
    output [7:0] gpio_out,

    output tx,
    input rx,

    output sck,
    output mosi,
    input miso,

    output flash_sck,
    output flash_cs,
    output flash_mosi,
    input flash_miso,

    input clk,
    input nreset
);

wire core_valid;
wire core_ready;
wire [3:0] core_we;
wire [31:0] core_Addr;
wire [31:0] core_Dout;
wire [31:0] core_Din;

wire rom_valid;
wire rom_en;
wire [31:0] rom_Addr;
wire [31:0] rom_Din;

wire from_valid;
wire from_ready;
wire [3:0] from_we;
wire [31:0] from_Addr;
wire [31:0] from_Dout;
wire [31:0] from_Din;

wire peripherie_valid;
wire peripherie_ready;
wire [3:0] peripherie_we;
wire [31:0] peripherie_Addr;
wire [31:0] peripherie_Dout;
wire [31:0] peripherie_Din;

wire gpio_valid;
wire gpio_ready;
wire [3:0] gpio_we;
wire [31:0] gpio_Addr;
wire [31:0] gpio_Dout;
wire [31:0] gpio_Din;
wire gpio_ir_request;

wire spi_valid;
wire spi_ready;
wire [3:0] spi_we;
wire [31:0] spi_Addr;
wire [31:0] spi_Dout;
wire [31:0] spi_Din;
wire spi_ir_request;

wire [31:0] ram_Addr;
wire [3:0] ram_we;
wire ram_en;
wire ram_write = |ram_we;
wire [31:0] ram_Dout;
wire [31:0] ram_Din;

wire uart_valid;
wire uart_ready;
wire [3:0] uart_we;
wire [31:0] uart_Addr;
wire [31:0] uart_Dout;
wire [31:0] uart_Din;
wire uart_tx_ir_flag;
wire uart_rx_ir_flag;
wire uart_oerr_ir_flag;
wire uart_perr_ir_flag;

wire ir_request;
wire ir_grant;
wire [31:0] ivec;
wire irc_valid;
wire irc_ready;
wire [3:0] irc_we;
wire [31:0] irc_Addr;
wire [31:0] irc_Dout;
wire [31:0] irc_Din;

Core #
(
   .RESET_ADDR( 32'h00000000 )
)
core
(
    //==System Bus
    .valid( core_valid ),
    .ready( core_ready ),
    .we( core_we ),
    .Addr( core_Addr ),
    .Dout( core_Dout ),
    .Din( core_Din ),

    .ir_request( ir_request ),
    .ir_grant( ir_grant ),
    .ivec( ivec ),

    //==Sync Signals
    .clk( clk ),
    .nreset( nreset )
);

Rom_256x32_t90 rom(
    .Q( rom_Din ),
    .CLK( clk ),
    .CEN( ~rom_en ),
    .A( rom_Addr[9:2] )
);

FlashCtrl from(
    //==Spi Interface
    .mosi( flash_mosi ),
    .miso( flash_miso ),
    .sck( flash_sck ),
    .cs( flash_cs ),

    //==System Bus Interface
    .Addr( from_Addr ),
    .valid( from_valid ),
    .ready( from_ready ),
    .we( from_we ),
    .Dout( from_Dout ),
    .Din( from_Din ),

    //==Sync Signals
    .clk( clk ),
    .nreset( nreset )
);

Gpio gpio(
    //==System Bus
    .Addr( gpio_Addr ),
    .valid( gpio_valid ),
    .ready( gpio_ready ),
    .we( gpio_we ),
    .Din( gpio_Din ),
    .Dout( gpio_Dout ),

    //==GPIO Interface
    .gpio_out( gpio_out ),
    .gpio_in( gpio_in ),

    //==Interrupt
    .ir_request( gpio_ir_request ),

    //==Sync
    .clk( clk ),
    .nreset( nreset )
);

Rf2p_192x32_t90 ram(
    //==Read Port A
    .AA( ram_Addr[9:2] ),
    .QA( ram_Din ),
    .CLKA( clk ),
    .CENA( ~(ram_en & (ram_we == 4'b0000) ) ),

    //==Write Port B
    .AB( ram_Addr[9:2] ),
    .DB( ram_Dout ),
    .CENB( ~(ram_en & (|ram_we))),
    .WENB( ~ram_we ),
    .CLKB( clk ),

    //==Self Adjusg
    .EMAA( 3'b000 ),
    .EMAB( 3'b000 )
);

Uart uart(
    //==Uart Signals
    .rx( rx ),
    .tx( tx ),

    .Addr( uart_Addr ),
    .valid( uart_valid ),
    .ready( uart_ready ),
    .we( uart_we ),
    .Din( uart_Din ),
    .Dout( uart_Dout ),

    .tx_ir_flag( uart_tx_ir_flag ),
    .rx_ir_flag( uart_rx_ir_flag ),
    .oerr_ir_flag( uart_oerr_ir_flag ),
    .perr_ir_flag( uart_perr_ir_flag ),

    //==Sync Signals
    .clk( clk ),
    .nreset( nreset )
);

Irc irc
(
    //==Interrupt Channels
    /*
    .ir_channel( {uart_perr_ir_flag, 
                  uart_oerr_ir_flag, 
                  uart_tx_ir_flag, 
                  uart_rx_ir_flag, 
                  gpio_ir_request} 
               ),
    */
    .ir_channel( {uart_tx_ir_flag, 
                  uart_rx_ir_flag, 
                  uart_perr_ir_flag, 
                  uart_oerr_ir_flag, 
                  gpio_ir_request} 
               ),

    //==Interrupt Core Interface
    .ir_request( ir_request ),
    .ir_grant( ir_grant ),
    .ivec( ivec ),

    //==System Bus
    .Addr( irc_Addr ),
    .valid( irc_valid ),
    .ready( irc_ready ),
    .we( irc_we ),
    .Dout( irc_Dout ),
    .Din( irc_Din ),

    //==Sync Interface
    .clk( clk ),
    .nreset( nreset )
);

Spi spi(
    //Spi Bus
    .sck( sck ),
    .mosi( mosi ),
    .miso( miso ),

    //==System Bus Interface
    .Addr( spi_Addr ),
    .valid( spi_valid ),
    .ready( spi_ready ),
    .we( spi_we ),
    .Dout( spi_Dout ),
    .Din( spi_Din ),

    .ir_request( spi_ir_request ),

    //==Sync Signals
    .clk( clk ),
    .nreset( nreset )
);

MemBusSwitch # 
(
    .M0_BASE_ADDR( 32'h00000000 ),
    .M0_ADDR_MASK( 32'hF8000000 ),

    .M1_BASE_ADDR( 32'h10000000 ),
    .M1_ADDR_MASK( 32'hF8000000 ),

    .M2_BASE_ADDR( 32'h08000000 ),
    .M2_ADDR_MASK( 32'hF8000000 )

)
MemBusSwitch
(
    .s0_Addr( core_Addr ),
    .s0_valid( core_valid ),
    .s0_ready( core_ready ),
    .s0_we( core_we ),
    .s0_Dout( core_Dout ),
    .s0_Din( core_Din ),

    .m0_Addr( rom_Addr ),
    .m0_valid( rom_en ),
    .m0_ready(),
    .m0_we(),
    .m0_Dout(),
    .m0_Din( rom_Din ),

    .m1_Addr( ram_Addr ),
    .m1_valid( ram_en ),
    .m1_ready(),
    .m1_we( ram_we ),
    .m1_Dout( ram_Dout ),
    .m1_Din( ram_Din),

    .m2_Addr( from_Addr ),
    .m2_valid( from_valid ),
    .m2_ready( from_ready ),
    .m2_we( from_we ),
    .m2_Dout( from_Dout ),
    .m2_Din( from_Din ),

    .md_Addr( peripherie_Addr ),
    .md_valid( peripherie_valid ),
    .md_ready( peripherie_ready ),
    .md_we( peripherie_we ),
    .md_Dout( peripherie_Dout ),
    .md_Din( peripherie_Din ),

    .clk( clk ),
    .nreset( nreset )
);

PeripherieBusSwitch # 
(
    .M0_BASE_ADDR( 32'h30000000 ),
    .M0_ADDR_MASK( 32'hf0000000 ),

    .M1_BASE_ADDR( 32'h20000000 ),
    .M1_ADDR_MASK( 32'hf0000000 ),

    .M2_BASE_ADDR( 32'h40000000 ),
    .M2_ADDR_MASK( 32'hf0000000 ),

    .M3_BASE_ADDR( 32'hf0000000 ),
    .M3_ADDR_MASK( 32'hf0000000 )
)
peripherieBusSwitch
(
    .s0_Addr( peripherie_Addr ),
    .s0_valid( peripherie_valid ),
    .s0_ready( peripherie_ready ),
    .s0_we( peripherie_we ),
    .s0_Dout( peripherie_Dout ),
    .s0_Din( peripherie_Din ),

    .m0_Addr( gpio_Addr ),
    .m0_valid( gpio_valid ),
    .m0_ready( gpio_ready ),
    .m0_we( gpio_we ),
    .m0_Dout( gpio_Dout ),
    .m0_Din( gpio_Din ),

    .m1_Addr( uart_Addr ),
    .m1_valid( uart_valid ),
    .m1_ready( uart_ready ),
    .m1_we( uart_we ),
    .m1_Dout( uart_Dout ),
    .m1_Din( uart_Din ),

    .m2_Addr( spi_Addr ),
    .m2_valid( spi_valid ),
    .m2_ready( spi_ready ),
    .m2_we( spi_we ),
    .m2_Dout( spi_Dout ),
    .m2_Din( spi_Din ),

    .m3_Addr( irc_Addr ),
    .m3_valid( irc_valid ),
    .m3_ready( irc_ready ),
    .m3_we( irc_we ),
    .m3_Dout( irc_Dout ),
    .m3_Din( irc_Din ),

    .clk( clk ),
    .nreset( nreset )
);

endmodule
