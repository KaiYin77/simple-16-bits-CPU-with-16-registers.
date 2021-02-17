module SCPU(
    // Input signals
    clk,
    rst_n,
    in_valid,
    instruction,
    MEM_out,
    // Output signals
    busy,
    out_valid,
    out0,
    out1,
    out2,
    out3,
    out4,
    out5,
    out6,
    out7,
    out8,
    out9,
    out10,
    out11,
    out12,
    out13,
    out14,
    out15,
    WEN,
    ADDR,
    MEM_in
);

//---------------------------------------------------------------------
//   INPUT AND OUTPUT DECLARATION
//---------------------------------------------------------------------
input clk, rst_n, in_valid;
input [18:0] instruction;
output reg busy, out_valid;
output reg signed [15:0] out0, out1, out2, out3, out4, out5, out6, out7, 
                         out8, out9, out10, out11, out12, out13, out14, out15;

input signed [15:0] MEM_out;
output reg WEN;
output reg signed [15:0] MEM_in;//Write to Memory
output reg [12:0] ADDR;         //Memory Address

//---------------------------------------------------------------------
//   WIRE AND REG DECLARATION
//---------------------------------------------------------------------
//Format_Definition
wire [2:0] opcode       = instruction[18:16];
wire [3:0] rs           = instruction[15:12];
wire [3:0] rt           = instruction[11:8];
wire [3:0] rd           = instruction[7:4];
wire [3:0] rl           = instruction[3:0];
wire [3:0] func         = instruction[3:0];
wire signed [7:0] imm   = instruction[7:0];

reg signed [31:0] ALUresult; //Store 16+16bit Alu result

//Data Hazard stall counter
reg[2:0] DH_counter1, DH_counter2;

//Data Hazard condition
reg conditions1, conditions2;

//Registers0~15 & rs,rt,rd,rl
reg signed [15:0] Reg [15:0], rs_value, rt_value, rd_value, rl_value;

//IF-ID reg/wire
reg [2:0] IFID_opcode, IFID_opcode_dh;
reg signed [15:0] IFIDrs_value, IFIDrt_value, IFIDrs_value_dh, IFIDrt_value_dh;
reg signed [7:0] IFID_imm, IFID_imm_dh;
reg IFID_out_valid;
reg [3:0] IFID_func, IFID_func_dh;
reg [3:0] IFID_rs ,IFID_rd, IFID_rl, IFID_rt, IFID_rs_dh, IFID_rd_dh, IFID_rl_dh, IFID_rt_dh;
wire [12:0] addr_base = IFIDrs_value[12:0];

//ID-EX reg
reg [2:0] IDEX_opcode;
reg IDEX_out_valid;
reg [3:0] IDEX_rs, IDEX_rd, IDEX_rl, IDEX_rt;
reg signed [31:0] IDEX_ALUresult;

//EX-MEM reg
reg [2:0] EXMEM_opcode;
reg EXMEM_out_valid;
reg [3:0] EXMEM_rs, EXMEM_rd, EXMEM_rl, EXMEM_rt;
reg signed [31:0] EXMEM_ALUresult;

//MEM-WB reg
reg MEMWB_out_valid;

//---------------------------------------------------------------------
//   Design Description
//---------------------------------------------------------------------

//Data_Hazard Detection Part
always@(clk) begin
    //2-stall cases 1st-dependency:load-use || R-type's Raw-data dependency 
    conditions2  =   (IFID_opcode  == 3'b110 && ((IFID_rt==rs)||(IFID_rt==rt))) ||
                     (IFID_opcode  == 3'b000 && ((IFID_rd==rs)||(IFID_rd==rt))) ||
                     (IFID_opcode  == 3'b001 && ((IFID_rd==rs)||(IFID_rd==rt)||(IFID_rl==rs)||(IFID_rl==rt)))||
                     ((IFID_opcode == 3'b010 || IFID_opcode == 3'b111)&& ((IFID_rd==rs)||(IFID_rd==rt)||(IFID_rl==rs)||(IFID_rl==rt)))||
                     ((IFID_opcode == 3'b011 || IFID_opcode == 3'b100)&& ((IFID_rt==rs)||(IFID_rt==rt)));
    //1-stall cases 2st-dependency:load-use || R-type's Raw-data dependency
    conditions1  =   (IDEX_opcode  == 3'b110 && ((IDEX_rt==rs)||(IDEX_rt==rt))) ||
                     (IDEX_opcode  == 3'b000 && ((IDEX_rd==rs)||(IDEX_rd==rt))) ||
                     (IDEX_opcode  == 3'b001 && ((IDEX_rd==rs)||(IDEX_rd==rt)||(IDEX_rl==rs)||(IDEX_rl==rt)))||
                     ((IDEX_opcode == 3'b010 || IDEX_opcode == 3'b111)&& ((IDEX_rd==rs)||(IDEX_rd==rt)||(IDEX_rl==rs)||(IDEX_rl==rt)))||
                     ((IDEX_opcode == 3'b011 || IDEX_opcode == 3'b100)&& ((IDEX_rt==rs)||(IDEX_rt==rt)));
end

//Combinational_Logic Part
always@(*) begin
    //Fetch rs&rd address
    case(rs)
        0: rs_value = Reg[0];  1: rs_value = Reg[1];  2: rs_value = Reg[2];  3: rs_value = Reg[3];
        4: rs_value = Reg[4];  5: rs_value = Reg[5];  6: rs_value = Reg[6];  7: rs_value = Reg[7];
        8: rs_value = Reg[8];  9: rs_value = Reg[9];  10: rs_value = Reg[10];  11: rs_value = Reg[11];
        12: rs_value = Reg[12];  13: rs_value = Reg[13];  14: rs_value = Reg[14];  15: rs_value = Reg[15];
    endcase
    case(rt)
        0: rt_value = Reg[0];  1: rt_value = Reg[1];  2: rt_value = Reg[2];  3: rt_value = Reg[3];
        4: rt_value = Reg[4];  5: rt_value = Reg[5];  6: rt_value = Reg[6];  7: rt_value = Reg[7];
        8: rt_value = Reg[8];  9: rt_value = Reg[9];  10: rt_value = Reg[10];  11: rt_value = Reg[11];
        12: rt_value = Reg[12];  13: rt_value = Reg[13];  14: rt_value = Reg[14];  15: rt_value = Reg[15];
    endcase
    //ALU combinational logic operation
    case (IFID_opcode)
        //R-type
        3'b000: begin
            case(IFID_func)
                4'b0000: ALUresult = IFIDrs_value & IFIDrt_value;
                4'b0001: ALUresult = IFIDrs_value | IFIDrt_value;
                4'b0010: ALUresult = IFIDrs_value ^ IFIDrt_value;
                4'b0011: ALUresult = IFIDrs_value + IFIDrt_value;
                4'b0100: ALUresult = IFIDrs_value - IFIDrt_value;
            endcase
        end
        //MULT
        3'b001: begin
            ALUresult = IFIDrs_value * IFIDrt_value;
        end
        //BEQ
        3'b010: begin
            if (IFIDrs_value == IFIDrt_value) begin
                ALUresult = 1;                  //(rd=1;rl=0)
            end
            else begin
                ALUresult = 0;                  //(rd=0;rl=1)
            end
        end
        //SLT
        3'b111: begin
            if (IFIDrs_value < IFIDrt_value) begin
               ALUresult = 1;                  //(rd=1;rl=0)
            end
            else begin
                ALUresult = 0;                  //(rd=0;rl=1)
            end
        end
        3'b011: begin
            ALUresult = IFIDrs_value + IFID_imm;
        end
        3'b100: begin
            ALUresult = IFIDrs_value - IFID_imm;
        end
        3'b101: begin
            ALUresult = addr_base + IFID_imm;       //the location of memory    
        end
        3'b110: begin
            ALUresult = addr_base + IFID_imm;       //the location of memory    
        end
        default: ALUresult = 0;
    endcase
end

//Re-Fetch IFID_rs's & IFID_rd's value after stall 
always@(DH_counter2 == 2 || DH_counter1 == 1)begin
    case(IFID_rs_dh)
        0: IFIDrs_value = Reg[0];  1: IFIDrs_value = Reg[1];  2: IFIDrs_value = Reg[2];  3: IFIDrs_value = Reg[3];
        4: IFIDrs_value = Reg[4];  5: IFIDrs_value = Reg[5];  6: IFIDrs_value = Reg[6];  7: IFIDrs_value = Reg[7];
        8: IFIDrs_value = Reg[8];  9: IFIDrs_value = Reg[9];  10: IFIDrs_value = Reg[10];  11: IFIDrs_value = Reg[11];
        12: IFIDrs_value = Reg[12];  13: IFIDrs_value = Reg[13];  14: IFIDrs_value = Reg[14];  15: IFIDrs_value = Reg[15];
    endcase
    case(IFID_rt_dh)
        0: IFIDrt_value = Reg[0];  1: IFIDrt_value = Reg[1];  2: IFIDrt_value = Reg[2];  3: IFIDrt_value = Reg[3];
        4: IFIDrt_value = Reg[4];  5: IFIDrt_value = Reg[5];  6: IFIDrt_value = Reg[6];  7: IFIDrt_value = Reg[7];
        8: IFIDrt_value = Reg[8];  9: IFIDrt_value = Reg[9];  10: IFIDrt_value = Reg[10];  11: IFIDrt_value = Reg[11];
        12: IFIDrt_value = Reg[12];  13: IFIDrt_value = Reg[13];  14: IFIDrt_value = Reg[14];  15: IFIDrt_value = Reg[15];
    endcase
    //ALU combinational logic operation
    case (IFID_opcode_dh)
        //R-type
        3'b000: begin
            case(IFID_func)
                4'b0000: ALUresult = IFIDrs_value & IFIDrt_value;
                4'b0001: ALUresult = IFIDrs_value | IFIDrt_value;
                4'b0010: ALUresult = IFIDrs_value ^ IFIDrt_value;
                4'b0011: ALUresult = IFIDrs_value + IFIDrt_value;
                4'b0100: ALUresult = IFIDrs_value - IFIDrt_value;
            endcase
        end
        //MULT
        3'b001: begin
            ALUresult = IFIDrs_value * IFIDrt_value;
        end
        //BEQ
        3'b010: begin
            if (IFIDrs_value == IFIDrt_value) begin
                ALUresult = 1;                  //(rd=1;rl=0)
            end
            else begin
                ALUresult = 0;                  //(rd=0;rl=1)
            end
        end
        //SLT
        3'b111: begin
            if (IFIDrs_value < IFIDrt_value) begin
               ALUresult = 1;                  //(rd=1;rl=0)
            end
            else begin
                ALUresult = 0;                  //(rd=0;rl=1)
            end
        end
        3'b011: begin
            ALUresult = IFIDrs_value + IFID_imm;
        end
        3'b100: begin
            ALUresult = IFIDrs_value - IFID_imm;
        end
        3'b101: begin
            ALUresult = addr_base + IFID_imm;       //the location of memory    
        end
        3'b110: begin
            ALUresult = addr_base + IFID_imm;       //the location of memory    
        end
        default: ALUresult = 0;
    endcase
end

//Sequential part
always@(posedge clk or negedge rst_n) begin
    //Dealing initial Reset case, Clearing all outputs and registers
    if (!rst_n) begin
        //outputs
        DH_counter1 <= 0;
        DH_counter2 <= 0;
        busy <= 0;
        MEM_in <= 0;
        WEN <= 1;
        ADDR <= 0;
        out_valid <= 0;
        out0 <= 0; 
        out1 <= 0; 
        out2 <= 0; 
        out3 <= 0; 
        out4 <= 0; 
        out5 <= 0; 
        out6 <= 0; 
        out7 <= 0;
        out8 <= 0; 
        out9 <= 0; 
        out10 <= 0; 
        out11 <= 0; 
        out12 <= 0; 
        out13 <= 0; 
        out14 <= 0; 
        out15 <= 0;  

        //Reg_value
        rs_value <= 0;
        rt_value <= 0;
        rd_value <= 0;
        rl_value <= 0;

        //Reg between stages
        IFID_out_valid <= 0;
        IDEX_out_valid <= 0;
        EXMEM_out_valid <= 0;
        MEMWB_out_valid <= 0;
        IFIDrs_value <= 0;
        IFIDrt_value <= 0;
        IFID_imm <= 0;
        IDEX_ALUresult <= 0;
        EXMEM_ALUresult <=0;
        IFID_opcode <= 0;
        IDEX_opcode <= 0;
        EXMEM_opcode <= 0;
        IFID_func <= 0;
        IFID_rs <= 0; IDEX_rs <= 0; EXMEM_rs <= 0;
        IFID_rd <= 0; IDEX_rd <= 0; EXMEM_rd <= 0;
        IFID_rl <= 0; IDEX_rl <= 0; EXMEM_rl <= 0;
        IFID_rt <= 0; IDEX_rt <= 0; EXMEM_rt <= 0; 
        ALUresult <= 0;

        //Initialize Registers
        Reg[0] <= 0;Reg[1] <= 0;Reg[2] <= 0;Reg[3] <= 0;Reg[4] <= 0;Reg[5] <= 0;
        Reg[6] <= 0;Reg[7] <= 0;Reg[8] <= 0;Reg[9] <= 0;Reg[10] <= 0;Reg[11] <= 0;
        Reg[12] <= 0; Reg[13] <= 0;Reg[14] <= 0;Reg[15] <= 0;
    end 
    //Sequential_Piplining Part
    else begin
        //1-stall cases => create 1 stall
        if((conditions1&&conditions2&&DH_counter2==0) || DH_counter2!=0) begin
            if(DH_counter2 == 2) begin
                busy <= 0;
                DH_counter2 <= 0;
                IFID_opcode <= IFID_opcode_dh;
                IFID_rs <= IFID_rs_dh;
                IFID_rt <= IFID_rt_dh;
                IFID_out_valid <= 1;
            end
            else if(DH_counter2 == 1) begin
                busy <= 1;
                DH_counter2 <= 2;
                IFID_opcode_dh <= IFID_opcode;
                IFID_rs <= IFID_rs_dh;
                IFID_rt <= IFID_rt_dh;
            end
            else if(DH_counter2 == 0) begin
                busy <= 1;
                DH_counter2 <= 1;
                IFID_out_valid <= 0;
                IFID_opcode <= opcode;
                IFID_opcode_dh <= opcode;
                IFID_imm <= imm;
                IFID_rs_dh <= rs;
                IFID_rt_dh <= rt;
                IFID_rd <= rd;
                IFID_rl <= rl;
                IFID_func <= func;
            end 
        end
        else if ((conditions1&&DH_counter1==0) || DH_counter1!=0) begin 
            if(DH_counter1==1) begin
                         busy <= 0;
                DH_counter1 <= 0;
                IFID_opcode <= IFID_opcode_dh;
                IFID_rs <= IFID_rs_dh;
                IFID_rt <= IFID_rt_dh;
                IFID_out_valid <= 1;
            end
            else if(DH_counter1==0) begin
                        busy <= 1;
                DH_counter1 <= 1;
                IFID_out_valid <= 0;
                IFID_opcode <= opcode;
                IFID_opcode_dh <= opcode;
                IFID_imm <= imm;
                IFID_rs_dh <= rs;
                IFID_rt_dh <= rt;
                IFID_rd <= rd;
                IFID_rl <= rl;
                IFID_func <= func;
            end
        end
        //2-stall cases => create 2 stall
        else if ((conditions2&&DH_counter2==0) || DH_counter2!=0) begin
        //Another method : always stall 2 cyles as hazard being detected 
        //if (conditions2 || conditions1 || DH_counter2!=0) begin
           if(DH_counter2 == 2) begin
                busy <= 0;
                DH_counter2 <= 0;
                IFID_opcode <= IFID_opcode_dh;
                IFID_rs <= IFID_rs_dh;
                IFID_rt <= IFID_rt_dh;
                IFID_out_valid <= 1;
            end
            else if(DH_counter2 == 1) begin
                busy <= 1;
                DH_counter2 <= 2;
                IFID_opcode_dh <= IFID_opcode;
                IFID_rs <= IFID_rs_dh;
                IFID_rt <= IFID_rt_dh;
            end
            else if(DH_counter2 == 0) begin
                busy <= 1;
                DH_counter2 <= 1;
                IFID_out_valid <= 0;
                IFID_opcode <= opcode;
                IFID_opcode_dh <= opcode;
                IFID_imm <= imm;
                IFID_rs_dh <= rs;
                IFID_rt_dh <= rt;
                IFID_rd <= rd;
                IFID_rl <= rl;
                IFID_func <= func;
            end 
        end
        else begin 
            busy <= 0;
            IFID_out_valid <= in_valid;
            IFID_opcode <= opcode;
            IFIDrs_value <= rs_value;
            IFIDrt_value <= rt_value;
            IFID_imm <= imm;
            IFID_rs <= rs;
            IFID_rt <= rt;
            IFID_rd <= rd;
            IFID_rl <= rl;
            IFID_func <= func;
        end
        //passing out_valid to the next stage
        IDEX_out_valid <= IFID_out_valid;
        EXMEM_out_valid <= IDEX_out_valid;
        MEMWB_out_valid <= EXMEM_out_valid;
        out_valid <= MEMWB_out_valid;       

        //passing output_result to the next stage
        out0 <= Reg[0];   out1 <= Reg[1];   out2 <= Reg[2];     out3 <= Reg[3];
        out4 <= Reg[4];   out5 <= Reg[5];   out6 <= Reg[6];     out7 <= Reg[7];
        out8 <= Reg[8];   out9 <= Reg[9];   out10 <= Reg[10];   out11 <= Reg[11];
        out12 <= Reg[12]; out13 <= Reg[13]; out14 <= Reg[14];   out15 <= Reg[15];

        //1st stage IF-ID
        //IFIDrs_value    <= rs_value;
        //IFIDrt_value    <= rt_value;
        //IFID_imm        <= imm;
        //IFID_rs         <= rs;
        //IFID_rt         <= rt;
        //IFID_rd         <= rd;
        //IFID_rl         <= rl;
        //IFID_func       <= func;

        //2nd stage ID-EX
        IDEX_opcode <= IFID_opcode;
        IDEX_rs     <= IFID_rs;
        IDEX_rt     <= IFID_rt;
        IDEX_rd     <= IFID_rd;
        IDEX_rl     <= IFID_rl;

        //3rd stage EX-MEM
        EXMEM_ALUresult <= IDEX_ALUresult;
        EXMEM_opcode    <= IDEX_opcode;
        EXMEM_rs        <= IDEX_rs;
        EXMEM_rt        <= IDEX_rt;
        EXMEM_rd        <= IDEX_rd;
        EXMEM_rl        <= IDEX_rl;

        //4th stage (Pre-MEM-WB)
        case (IFID_opcode)
            3'b000:
            begin
                IDEX_ALUresult <= ALUresult;
            end
            3'b001:
            begin
                IDEX_ALUresult <= ALUresult;
            end
            3'b010:
            begin
                IDEX_ALUresult <= ALUresult; 
            end
            3'b011:
            begin
                IDEX_ALUresult <= ALUresult; 
            end
            3'b100:
            begin
                IDEX_ALUresult <= ALUresult; 
            end
            3'b101:
            begin
                WEN     <= 0;                  //store to memory
                ADDR    <= ALUresult;          //give memory address
                MEM_in  <= IFIDrt_value;       //give value to store in memory
            end
            3'b110:
            begin
                WEN <= 1;
                ADDR <= ALUresult;          //give memory address
            end
            default: IDEX_ALUresult <= ALUresult;
        endcase
        //4th stage (MEM-WB)
        case (EXMEM_opcode)
            //R-type needs write back to rd
            3'b000: begin  
                case (EXMEM_rd)
                    0:  Reg[0]   <=  EXMEM_ALUresult[15:0];
                    1:  Reg[1]   <=  EXMEM_ALUresult[15:0];
                    2:  Reg[2]   <=  EXMEM_ALUresult[15:0];
                    3:  Reg[3]   <=  EXMEM_ALUresult[15:0];
                    4:  Reg[4]   <=  EXMEM_ALUresult[15:0];
                    5:  Reg[5]   <=  EXMEM_ALUresult[15:0];
                    6:  Reg[6]   <=  EXMEM_ALUresult[15:0];
                    7:  Reg[7]   <=  EXMEM_ALUresult[15:0];
                    8:  Reg[8]   <=  EXMEM_ALUresult[15:0];
                    9:  Reg[9]   <=  EXMEM_ALUresult[15:0];
                    10: Reg[10]  <=  EXMEM_ALUresult[15:0];
                    11: Reg[11]  <=  EXMEM_ALUresult[15:0];
                    12: Reg[12]  <=  EXMEM_ALUresult[15:0];
                    13: Reg[13]  <=  EXMEM_ALUresult[15:0];
                    14: Reg[14]  <=  EXMEM_ALUresult[15:0];
                    15: Reg[15]  <=  EXMEM_ALUresult[15:0];
                endcase 
            end
            //Mult rd is result_high_15 ;rl is result_low_15
            3'b001: begin
                case (EXMEM_rd)
                    0:  Reg[0]   <=  EXMEM_ALUresult[31:16];
                    1:  Reg[1]   <=  EXMEM_ALUresult[31:16];
                    2:  Reg[2]   <=  EXMEM_ALUresult[31:16];
                    3:  Reg[3]   <=  EXMEM_ALUresult[31:16];
                    4:  Reg[4]   <=  EXMEM_ALUresult[31:16];
                    5:  Reg[5]   <=  EXMEM_ALUresult[31:16];
                    6:  Reg[6]   <=  EXMEM_ALUresult[31:16];
                    7:  Reg[7]   <=  EXMEM_ALUresult[31:16];
                    8:  Reg[8]   <=  EXMEM_ALUresult[31:16];
                    9:  Reg[9]   <=  EXMEM_ALUresult[31:16];
                    10: Reg[10]  <=  EXMEM_ALUresult[31:16];
                    11: Reg[11]  <=  EXMEM_ALUresult[31:16];
                    12: Reg[12]  <=  EXMEM_ALUresult[31:16];
                    13: Reg[13]  <=  EXMEM_ALUresult[31:16];
                    14: Reg[14]  <=  EXMEM_ALUresult[31:16];
                    15: Reg[15]  <=  EXMEM_ALUresult[31:16];
                endcase
                case (EXMEM_rl)
                    0:  Reg[0]   <=  EXMEM_ALUresult[15:0];
                    1:  Reg[1]   <=  EXMEM_ALUresult[15:0];
                    2:  Reg[2]   <=  EXMEM_ALUresult[15:0];
                    3:  Reg[3]   <=  EXMEM_ALUresult[15:0];
                    4:  Reg[4]   <=  EXMEM_ALUresult[15:0];
                    5:  Reg[5]   <=  EXMEM_ALUresult[15:0];
                    6:  Reg[6]   <=  EXMEM_ALUresult[15:0];
                    7:  Reg[7]   <=  EXMEM_ALUresult[15:0];
                    8:  Reg[8]   <=  EXMEM_ALUresult[15:0];
                    9:  Reg[9]   <=  EXMEM_ALUresult[15:0];
                    10: Reg[10]  <=  EXMEM_ALUresult[15:0];
                    11: Reg[11]  <=  EXMEM_ALUresult[15:0];
                    12: Reg[12]  <=  EXMEM_ALUresult[15:0];
                    13: Reg[13]  <=  EXMEM_ALUresult[15:0];
                    14: Reg[14]  <=  EXMEM_ALUresult[15:0];
                    15: Reg[15]  <=  EXMEM_ALUresult[15:0];
                endcase 
            end
            //BEQ eqaul? yes -> ALUresult = 1 -> rd = 1 & rt = 0;
            //BEQ equal? no  -> ALUresult = 0 -> rd = 0 & rt = 1;
            3'b010: begin
                if (EXMEM_ALUresult == 1) begin
                    case (EXMEM_rd)
                        0:  Reg[0]   <=  1;
                        1:  Reg[1]   <=  1;
                        2:  Reg[2]   <=  1;
                        3:  Reg[3]   <=  1;
                        4:  Reg[4]   <=  1;
                        5:  Reg[5]   <=  1;
                        6:  Reg[6]   <=  1;
                        7:  Reg[7]   <=  1;
                        8:  Reg[8]   <=  1;
                        9:  Reg[9]   <=  1;
                        10: Reg[10]  <=  1;
                        11: Reg[11]  <=  1;
                        12: Reg[12]  <=  1;
                        13: Reg[13]  <=  1;
                        14: Reg[14]  <=  1;
                        15: Reg[15]  <=  1;
                    endcase
                    case (EXMEM_rl)
                        0:  Reg[0]   <=  0;
                        1:  Reg[1]   <=  0;
                        2:  Reg[2]   <=  0;
                        3:  Reg[3]   <=  0;
                        4:  Reg[4]   <=  0;
                        5:  Reg[5]   <=  0;
                        6:  Reg[6]   <=  0;
                        7:  Reg[7]   <=  0;
                        8:  Reg[8]   <=  0;
                        9:  Reg[9]   <=  0;
                        10: Reg[10]  <=  0;
                        11: Reg[11]  <=  0;
                        12: Reg[12]  <=  0;
                        13: Reg[13]  <=  0;
                        14: Reg[14]  <=  0;
                        15: Reg[15]  <=  0;
                    endcase
                end
                else begin
                    case (EXMEM_rd)
                        0:  Reg[0]   <=  0;
                        1:  Reg[1]   <=  0;
                        2:  Reg[2]   <=  0;
                        3:  Reg[3]   <=  0;
                        4:  Reg[4]   <=  0;
                        5:  Reg[5]   <=  0;
                        6:  Reg[6]   <=  0;
                        7:  Reg[7]   <=  0;
                        8:  Reg[8]   <=  0;
                        9:  Reg[9]   <=  0;
                        10: Reg[10]  <=  0;
                        11: Reg[11]  <=  0;
                        12: Reg[12]  <=  0;
                        13: Reg[13]  <=  0;
                        14: Reg[14]  <=  0;
                        15: Reg[15]  <=  0;
                    endcase
                    case (EXMEM_rl)
                        0:  Reg[0]   <=  1;
                        1:  Reg[1]   <=  1;
                        2:  Reg[2]   <=  1;
                        3:  Reg[3]   <=  1;
                        4:  Reg[4]   <=  1;
                        5:  Reg[5]   <=  1;
                        6:  Reg[6]   <=  1;
                        7:  Reg[7]   <=  1;
                        8:  Reg[8]   <=  1;
                        9:  Reg[9]   <=  1;
                        10: Reg[10]  <=  1;
                        11: Reg[11]  <=  1;
                        12: Reg[12]  <=  1;
                        13: Reg[13]  <=  1;
                        14: Reg[14]  <=  1;
                        15: Reg[15]  <=  1;
                    endcase
                end
            end 
            //SLT rs_value < rt_value ? yes  -> ALUresult = 1 -> rd = 1 & rt = 0;
            //SLT rs_value < rt_value ? no   -> ALUresult = 0 -> rd = 0 & rt = 1;
            3'b111: begin
                if (EXMEM_ALUresult == 1) begin
                    case (EXMEM_rd)
                        0:  Reg[0]   <=  1;
                        1:  Reg[1]   <=  1;
                        2:  Reg[2]   <=  1;
                        3:  Reg[3]   <=  1;
                        4:  Reg[4]   <=  1;
                        5:  Reg[5]   <=  1;
                        6:  Reg[6]   <=  1;
                        7:  Reg[7]   <=  1;
                        8:  Reg[8]   <=  1;
                        9:  Reg[9]   <=  1;
                        10: Reg[10]  <=  1;
                        11: Reg[11]  <=  1;
                        12: Reg[12]  <=  1;
                        13: Reg[13]  <=  1;
                        14: Reg[14]  <=  1;
                        15: Reg[15]  <=  1;
                    endcase
                    case (EXMEM_rl)
                        0:  Reg[0]   <=  0;
                        1:  Reg[1]   <=  0;
                        2:  Reg[2]   <=  0;
                        3:  Reg[3]   <=  0;
                        4:  Reg[4]   <=  0;
                        5:  Reg[5]   <=  0;
                        6:  Reg[6]   <=  0;
                        7:  Reg[7]   <=  0;
                        8:  Reg[8]   <=  0;
                        9:  Reg[9]   <=  0;
                        10: Reg[10]  <=  0;
                        11: Reg[11]  <=  0;
                        12: Reg[12]  <=  0;
                        13: Reg[13]  <=  0;
                        14: Reg[14]  <=  0;
                        15: Reg[15]  <=  0;
                    endcase
                end
                else begin
                    case (EXMEM_rd)
                        0:  Reg[0]   <=  0;
                        1:  Reg[1]   <=  0;
                        2:  Reg[2]   <=  0;
                        3:  Reg[3]   <=  0;
                        4:  Reg[4]   <=  0;
                        5:  Reg[5]   <=  0;
                        6:  Reg[6]   <=  0;
                        7:  Reg[7]   <=  0;
                        8:  Reg[8]   <=  0;
                        9:  Reg[9]   <=  0;
                        10: Reg[10]  <=  0;
                        11: Reg[11]  <=  0;
                        12: Reg[12]  <=  0;
                        13: Reg[13]  <=  0;
                        14: Reg[14]  <=  0;
                        15: Reg[15]  <=  0;
                    endcase
                    case (EXMEM_rl)
                        0:  Reg[0]   <=  1;
                        1:  Reg[1]   <=  1;
                        2:  Reg[2]   <=  1;
                        3:  Reg[3]   <=  1;
                        4:  Reg[4]   <=  1;
                        5:  Reg[5]   <=  1;
                        6:  Reg[6]   <=  1;
                        7:  Reg[7]   <=  1;
                        8:  Reg[8]   <=  1;
                        9:  Reg[9]   <=  1;
                        10: Reg[10]  <=  1;
                        11: Reg[11]  <=  1;
                        12: Reg[12]  <=  1;
                        13: Reg[13]  <=  1;
                        14: Reg[14]  <=  1;
                        15: Reg[15]  <=  1;
                    endcase
                end
            end
            //ADDI result shoud write back to rt
            3'b011: begin
                case (EXMEM_rt)
                    0:  Reg[0]   <=  EXMEM_ALUresult[15:0];
                    1:  Reg[1]   <=  EXMEM_ALUresult[15:0];
                    2:  Reg[2]   <=  EXMEM_ALUresult[15:0];
                    3:  Reg[3]   <=  EXMEM_ALUresult[15:0];
                    4:  Reg[4]   <=  EXMEM_ALUresult[15:0];
                    5:  Reg[5]   <=  EXMEM_ALUresult[15:0];
                    6:  Reg[6]   <=  EXMEM_ALUresult[15:0];
                    7:  Reg[7]   <=  EXMEM_ALUresult[15:0];
                    8:  Reg[8]   <=  EXMEM_ALUresult[15:0];
                    9:  Reg[9]   <=  EXMEM_ALUresult[15:0];
                    10: Reg[10]  <=  EXMEM_ALUresult[15:0];
                    11: Reg[11]  <=  EXMEM_ALUresult[15:0];
                    12: Reg[12]  <=  EXMEM_ALUresult[15:0];
                    13: Reg[13]  <=  EXMEM_ALUresult[15:0];
                    14: Reg[14]  <=  EXMEM_ALUresult[15:0];
                    15: Reg[15]  <=  EXMEM_ALUresult[15:0];
                endcase
            end
            //SUBI result should write back to rt
            3'b100:begin
                case (EXMEM_rt)
                    0:  Reg[0]   <=  EXMEM_ALUresult[15:0];
                    1:  Reg[1]   <=  EXMEM_ALUresult[15:0];
                    2:  Reg[2]   <=  EXMEM_ALUresult[15:0];
                    3:  Reg[3]   <=  EXMEM_ALUresult[15:0];
                    4:  Reg[4]   <=  EXMEM_ALUresult[15:0];
                    5:  Reg[5]   <=  EXMEM_ALUresult[15:0];
                    6:  Reg[6]   <=  EXMEM_ALUresult[15:0];
                    7:  Reg[7]   <=  EXMEM_ALUresult[15:0];
                    8:  Reg[8]   <=  EXMEM_ALUresult[15:0];
                    9:  Reg[9]   <=  EXMEM_ALUresult[15:0];
                    10: Reg[10]  <=  EXMEM_ALUresult[15:0];
                    11: Reg[11]  <=  EXMEM_ALUresult[15:0];
                    12: Reg[12]  <=  EXMEM_ALUresult[15:0];
                    13: Reg[13]  <=  EXMEM_ALUresult[15:0];
                    14: Reg[14]  <=  EXMEM_ALUresult[15:0];
                    15: Reg[15]  <=  EXMEM_ALUresult[15:0];
                endcase
            end
            //LOAD result should write back to rt
            3'b110: begin
                case (EXMEM_rt)
                    0:  Reg[0]   <=  MEM_out;
                    1:  Reg[1]   <=  MEM_out;
                    2:  Reg[2]   <=  MEM_out;
                    3:  Reg[3]   <=  MEM_out;
                    4:  Reg[4]   <=  MEM_out;
                    5:  Reg[5]   <=  MEM_out;
                    6:  Reg[6]   <=  MEM_out;
                    7:  Reg[7]   <=  MEM_out;
                    8:  Reg[8]   <=  MEM_out;
                    9:  Reg[9]   <=  MEM_out;
                    10: Reg[10]  <=  MEM_out;
                    11: Reg[11]  <=  MEM_out;
                    12: Reg[12]  <=  MEM_out;
                    13: Reg[13]  <=  MEM_out;
                    14: Reg[14]  <=  MEM_out;
                    15: Reg[15]  <=  MEM_out;
                endcase
            end
            default: begin
                Reg[0]   <=  Reg[0];
                Reg[1]   <=  Reg[1];
                Reg[2]   <=  Reg[2];
                Reg[3]   <=  Reg[3];
                Reg[4]   <=  Reg[4];
                Reg[5]   <=  Reg[5];
                Reg[6]   <=  Reg[6];
                Reg[7]   <=  Reg[7];
                Reg[8]   <=  Reg[8];
                Reg[9]   <=  Reg[9];
                Reg[10]  <=  Reg[10];
                Reg[11]  <=  Reg[11];
                Reg[12]  <=  Reg[12];
                Reg[13]  <=  Reg[13];
                Reg[14]  <=  Reg[14];
                Reg[15]  <=  Reg[15];
            end
        endcase
    end
end

endmodule