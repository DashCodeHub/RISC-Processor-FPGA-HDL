`timescale 1ns / 1ps

module risc_fpga_(
    input [15:0] SW, // this is the reset option given from FPGA switch (SW[0] )
    input clk_i,
    output [7:0] disp_an_o, // anodes to select display Should always be 1110000
    output [7:0] disp_seg_o // Cathodes to select segment
    //output reg [15:0] LED // LED to show for clock
); 
    wire [15:0] mem_loc_201;
    wire [15:0] mem_loc_202;
    wire [15:0] mem_loc_203;
    wire [15:0] mem_loc_204;
    wire [15:0] mem_loc_205;
    
    // Get the values of the memory location from RISC FPGA
    risc_machine machine(.clk(clk_i), .rst(SW[0]));

    // assing the memory values to the memory location
    assign mem_loc_201 = machine.MU.memory256x16[201];
    assign mem_loc_202 = machine.MU.memory256x16[202];
    assign mem_loc_203 = machine.MU.memory256x16[203];
    assign mem_loc_204 = machine.MU.memory256x16[204];
    assign mem_loc_205 = machine.MU.memory256x16[205];


    reg [32:0] counter; // 17-bit counter to divide 100MHDZ Clock into 1KHz
    reg [2:0] select; // Selector for the mux (3 bits because only need 5) 
    always @(posedge clk_i) begin 
        if (counter != 100_000_000) begin // Make it 100_000_000 when running in fpga else keep 2_000_000
            counter <= counter + 1;
        end
        else begin 
            counter <= 0;
            //LED <= ~LED;
            select <= (select < 4)? select+1:0; //loop through 0 to 4
        end
    end


    // MUX to send values
    reg [15:0] mem_to_led;

    //MUX to send value according to select1 (1HZ)
    always @(*) begin
        // use case statement 
        case (select)
            3'd0: begin 
                mem_to_led = mem_loc_201;
            end
            3'd1: begin
                mem_to_led = mem_loc_202;
            end
            3'd2: begin
                mem_to_led = mem_loc_203;
            end
            3'd3: begin
                mem_to_led = mem_loc_204;
            end
            3'd4: begin
                mem_to_led = mem_loc_205;
            end
            default: begin 
                mem_to_led = 16'bx;
            end
        endcase
    end

    led_segment segment(
        .clk_i(clk_i),
        .bin_inp(mem_to_led),
        .anodes(disp_an_o), // anodes to select display Should always be 1110000
        .cathodes(disp_seg_o) // Cathodes to select segment
        //output reg [15:0] LED // LED to show for clock
    );
endmodule

module bin2bcd(
    input [15:0] binary,
    output reg [19:0] bcd
);

    integer i;

    always @(binary) begin
        // Initialize the BCD output to 0
        bcd = 20'b0;
        
        // Iterate over each bit in the binary input
        for (i = 15; i >= 0; i = i - 1) begin
            // Add 3 to each BCD digit if it is greater than or equal to 5
            if (bcd[3:0] > 4) bcd[3:0] = bcd[3:0] + 3;
            if (bcd[7:4] > 4) bcd[7:4] = bcd[7:4] + 3;
            if (bcd[11:8] > 4) bcd[11:8] = bcd[11:8] + 3;
            if (bcd[15:12] > 4) bcd[15:12] = bcd[15:12] + 3;
            if (bcd[19:16] > 4) bcd[19:16] = bcd[19:16] + 3;
            // Shift the current BCD value left by one bit
            bcd = {bcd[18:0], binary[i]};
        end
    end

endmodule

module SEVENSEGMENT_1x8(
    input [3:0] inp,
    output reg [7:0] seg
);
    always@(inp) begin 
        case (inp) 
            4'd0: seg = 8'b11000000;
            4'd1: seg = 8'b11111001;
            4'd2: seg = 8'b10100100;
            4'd3: seg = 8'b10110000;
            4'd4: seg = 8'b10011001;
            4'd5: seg = 8'b10010010;
            4'd6: seg = 8'b10000010;
            4'd7: seg = 8'b11111000;
            4'd8: seg = 8'b10000000;
            4'd9: seg = 8'b10010000;
            default seg = 8'b11111111;
        endcase
    end
endmodule

module led_segment(
    input clk_i,
    input [15:0] bin_inp,
    output reg [7:0] anodes, // anodes to select display Should always be 1110000
    output reg [7:0] cathodes // Cathodes to select segment
    //output reg [15:0] LED // LED to show for clock
);
    

    //Get the BCD 
    wire [19:0] bcd;
    bin2bcd uut(
        .binary(bin_inp),
        .bcd(bcd)
    );
    //get the value for sending to each LED Segment
    //5 outputs so 5 reg each of 8 bit
    wire [7:0] seg0;
    wire [7:0] seg1;
    wire [7:0] seg2;
    wire [7:0] seg3;
    wire [7:0] seg4;
    //get the values for each segment (calling 5 times)
    // For seg 0
    SEVENSEGMENT_1x8 ledseg0(
        .inp(bcd[3:0]),
        .seg(seg0)
    );
    //For seg 1
    SEVENSEGMENT_1x8 ledseg1(
        .inp(bcd[7:4]),
        .seg(seg1)
    );
    //For seg 2
    SEVENSEGMENT_1x8 ledseg2(
        .inp(bcd[11:8]),
        .seg(seg2)
    );
    //For seg 3
    SEVENSEGMENT_1x8 ledseg3(
        .inp(bcd[15:12]),
        .seg(seg3)
    );
    //For seg 4
    SEVENSEGMENT_1x8 ledseg4(
        .inp(bcd[19:16]),
        .seg(seg4)
    );

    
    //make a 1kHz clock input
    reg [16:0] counter; // 17-bit counter to divide 100MHDZ Clock into 1KHz
    reg [2:0] select; // Selector for the mux (3 bits because only need 5) 
    always @(posedge clk_i) begin 
        if (counter != 50000) begin 
            counter <= counter + 1;
        end
        else begin 
            counter <= 0;
            //LED <= ~LED;
            select <= (select < 4)? select+1:0; //loop through 0 to 4
        end
    end

    always @(*) begin
        // use case statement 
        case (select)
            3'd0: begin 
                anodes <= 8'b11111110; //display the AN0
                cathodes <= seg0;
            end
            3'd1: begin
                anodes <= 8'b11111101; //display the AN1
                cathodes <= seg1;
            end
            3'd2: begin
                anodes <= 8'b11111011; //display the AN2 
                cathodes <= seg2;
            end
            3'd3: begin
                anodes <= 8'b11110111;  //display the AN3 
                cathodes <= seg3;
            end
            3'd4: begin
                anodes <= 8'b11101111; //display the AN4
                cathodes <= seg4;
            end
            default: begin 
                anodes <= 8'b11111111; // all display not used
            end
        endcase
    end
endmodule


module risc_machine (clk, rst);
    // parameters description
    parameter data_width = 16; // Width of all data
    parameter addr_width = 8; // Address width for memory and Program Counter
    parameter RF_W_Addr_Width = 4; //Register file address width
    parameter RF_Depth = 16; // Register File Depth


    parameter opcode_size = 4; // Opcode Size
    parameter sel_bus_1_size = 5; //select control width
    parameter sel_bus_2_size = 2; // select control width

    //input output declaration
    input clk, rst;

    // Selects for Buses
    wire [sel_bus_1_size-1:0] Sel_Bus_1_MUX; // Selct BUS_1 Control Signal
    wire [sel_bus_2_size-1:0] Sel_Bus_2_MUX; // Select BUS_2 Control Signal
    
    // Data Nets
    wire alu_zero;
    wire RF_Ry_Zero;
    wire [data_width-1:0] instruction;
    wire [addr_width-1:0] address;
    wire [data_width-1:0] Bus_1;
    wire [data_width-1:0] mem_read_data; // input data from the memory

    // Control Nets
    wire PC_Ld; 
    wire PC_Inc; 
    wire sel_PC_Offset_Update; 
    wire Sign_Ext_Flag; 
    wire IR_Ld; 
    wire Reg_Y_Ld; 
    wire Reg_A_Ld; 
    wire Reg_Z_Ld; 
    wire [RF_W_Addr_Width-1:0] RF_W_Addr;
    wire D_wr; // data write enable signal

    //ProcessingUnit
    ProcessingUnit PU (instruction, RF_Ry_Zero, alu_zero, Bus_1, address, mem_read_data, RF_W_Addr, PC_Ld, PC_Inc, sel_PC_Offset_Update, Sel_Bus_1_MUX, Sign_Ext_Flag, IR_Ld, Reg_Y_Ld, Sel_Bus_2_MUX, Reg_A_Ld, Reg_Z_Ld, clk, rst);

    // Control Unit
    ControlUnit2 CU(
        PC_Ld, PC_Inc, sel_PC_Offset_Update, // Control Signals for PC
        RF_W_Addr,  // Address to write into Register File
        IR_Ld, // Control Signals to Load Instruction Register
        Sign_Ext_Flag, // Control Signal for Sign Extension immediate
        Sel_Bus_1_MUX, // Control Bus_1 MUX1
        Reg_Y_Ld, // Control Signal for Reg_Y
        RF_Ry_Zero, //Output Flag to check operand in RegY is 0 or not
        instruction, //Output instrution bytes
        Sel_Bus_2_MUX, // Control Bus_2 MUX2
        Reg_A_Ld, // Control Signal to Load Reg_A (Address_Register)
        Reg_Z_Ld, // Control Signal to Load Reg_Z (Zero ALU Flag)
        alu_zero, //Output flag for alu_output is 0 or not
        D_wr, // Data Write Enable signal to read memory
        clk, rst);

    // Memory Unit 
    main_memory MU(
        .W_data(Bus_1),
        .R_data(mem_read_data),
        .wr(D_wr),
        .addr(address),
        .clk(clk)
    );
    
endmodule

module ProcessingUnit(instruction, RF_Ry_Zero, alu_zero, Bus_1, address, mem_read_data, RF_W_Addr, PC_Ld, PC_Inc, sel_PC_Offset_Update, Sel_Bus_1_MUX, Sign_Ext_Flag, IR_Ld, Reg_Y_Ld, Sel_Bus_2_MUX, Reg_A_Ld, Reg_Z_Ld, clk, rst );

    // Define parameter
    parameter data_width = 16; // Width of all data
    parameter addr_width = 8; // Address width for memory and Program Counter
    parameter RF_W_Addr_Width = 4; //Register file address width
    parameter RF_Depth = 16; // Register File Depth


    parameter opcode_size = 4; // Opcode Size
    parameter sel_bus_1_size = 5; //select control width
    parameter sel_bus_2_size = 2; // select control width

    // Input and output declaration
    output [data_width-1:0] instruction; // instruction sent to contol unit to decode
    output [addr_width-1:0] address; // Output address for memory read
    output [data_width-1:0] Bus_1; // Bus 1
    output RF_Ry_Zero; // Zero Flag for Operand in Reg_Y
    output alu_zero; // Zero flag from ALU
    
    
    input [data_width-1:0] mem_read_data; // input data from the memory
    input [RF_W_Addr_Width-1:0] RF_W_Addr; // input Reg File Address Control Signal to read register file 
    input PC_Ld; // Load PC Control Signal *To be used while BIZ, BNZ, JAL, JMP ,JR to update PC with Offset
    input PC_Inc; // Incremenet PC Control Signal
    input sel_PC_Offset_Update; // Select between offset update (a+b-1) or data from BUS_2
    input [sel_bus_1_size-1:0] Sel_Bus_1_MUX; // Selct BUS_1 Control Signal
    input Sign_Ext_Flag; // Control signal to do sign extension immediate
    input IR_Ld; // Instruction Register Load control signal
    input Reg_Y_Ld; // Load Reg Y Control Signal
    input [sel_bus_2_size-1:0] Sel_Bus_2_MUX; // Select BUS_2 Control Signal
    input Reg_A_Ld; // Load Reg_A control Signal
    input Reg_Z_Ld; // Load Reg_Z control Signal
    input clk, rst; // CLOCK and RESET inputs

    // Temporary Intermediate wires
    wire [RF_Depth-1:0] load_BUS_to_RF; // Wire in between decoder and Register File
    wire [RF_Depth-1:0] R0_out, R1_out, R2_out, R3_out, R4_out, R5_out, R6_out, R7_out, R8_out, R9_out, R10_out, R11_out, R12_out, R13_out, R14_out, R15_out; // Ouptuts from Register File to 18x1 MUX [MUX1]
    wire [data_width-1:0] Bus_2; // Bus 2 for connection between hardwares inside the the processor unit
    wire [data_width-1:0] PC_addr; // Output from Program Counter
    wire [data_width-1:0] PC_data_in; // Input to Program Counter from 2x1 MUX3
    wire [data_width-1:0] Offset_PC_addr; // new PC address after calculation of PC_addr + IR[7:0](offset) - 1
    wire [data_width-1:0] Offset = {8'b0, instruction[7:0]} ; // Offset Output from IR
    wire [data_width-1:0] Imm_ext_data; // Signed Immediate data calculate
    wire [data_width-1:0] Reg_Y_Out; // Ouptut from Reg_Y to ALU
    wire [data_width-1:0] alu_out; // output from ALU
    wire [opcode_size-1:0] opcode = instruction[data_width-1:data_width-opcode_size]; // OPcode
    wire alu_zero_flag;
    

    // Connect a Register File Using Register File with decoder inbetween them 
    decoder_4_to_16 decode_RF_Address (
        .binary_in(RF_W_Addr),  // 4-bit binary input
        .dec_out(load_BUS_to_RF)    // 16-bit decoder output
    );
    // all registers in register file (16x16)
    RegisterUnit R0 (.data_out(R0_out), .data_in(Bus_2), .load(load_BUS_to_RF[0]), .clk(clk), .rst(rst));
    RegisterUnit R1 (.data_out(R1_out), .data_in(Bus_2), .load(load_BUS_to_RF[1]), .clk(clk), .rst(rst));
    RegisterUnit R2 (.data_out(R2_out), .data_in(Bus_2), .load(load_BUS_to_RF[2]), .clk(clk), .rst(rst));
    RegisterUnit R3 (.data_out(R3_out), .data_in(Bus_2), .load(load_BUS_to_RF[3]), .clk(clk), .rst(rst));
    RegisterUnit R4 (.data_out(R4_out), .data_in(Bus_2), .load(load_BUS_to_RF[4]), .clk(clk), .rst(rst));
    RegisterUnit R5 (.data_out(R5_out), .data_in(Bus_2), .load(load_BUS_to_RF[5]), .clk(clk), .rst(rst));
    RegisterUnit R6 (.data_out(R6_out), .data_in(Bus_2), .load(load_BUS_to_RF[6]), .clk(clk), .rst(rst));
    RegisterUnit R7 (.data_out(R7_out), .data_in(Bus_2), .load(load_BUS_to_RF[7]), .clk(clk), .rst(rst));
    RegisterUnit R8 (.data_out(R8_out), .data_in(Bus_2), .load(load_BUS_to_RF[8]), .clk(clk), .rst(rst));
    RegisterUnit R9 (.data_out(R9_out), .data_in(Bus_2), .load(load_BUS_to_RF[9]), .clk(clk), .rst(rst));
    RegisterUnit R10 (.data_out(R10_out), .data_in(Bus_2), .load(load_BUS_to_RF[10]), .clk(clk), .rst(rst));
    RegisterUnit R11 (.data_out(R11_out), .data_in(Bus_2), .load(load_BUS_to_RF[11]), .clk(clk), .rst(rst));
    RegisterUnit R12 (.data_out(R12_out), .data_in(Bus_2), .load(load_BUS_to_RF[12]), .clk(clk), .rst(rst));
    RegisterUnit R13 (.data_out(R13_out), .data_in(Bus_2), .load(load_BUS_to_RF[13]), .clk(clk), .rst(rst));
    RegisterUnit R14 (.data_out(R14_out), .data_in(Bus_2), .load(load_BUS_to_RF[14]), .clk(clk), .rst(rst));
    RegisterUnit R15 (.data_out(R15_out), .data_in(Bus_2), .load(load_BUS_to_RF[15]), .clk(clk), .rst(rst));

    // Instantiate The PC
    ProgramCounter PC(
        .ctr(PC_addr), 
        .up(PC_Inc), 
        .clr(rst), 
        .load(PC_Ld), 
        .clk(clk), 
        .data_in(PC_data_in)
    );
    // MUX for PC load data select
    MUX2x1 LoadPC_Data(
        .s(sel_PC_Offset_Update),
        .data_0(Offset_PC_addr),
        .data_1(Bus_2),
        .data_out(PC_data_in)
    );
    // Offset calculation
    compute_offset PC_Offset(
        .A(PC_addr), // PC Address
        .B(Offset), // Offset from IR[7:0] given as {8'b0, instruction[7:0]}
        .new_A(Offset_PC_addr) // new PC address to update 
    );
    //Instruction Register
    InstructionRegister IR(
        .instr_in(Bus_2), // Input of instruction through Bus_2, Given Wrong in book
        .IR_ld(IR_Ld), 
        .instr_out(instruction), 
        .clk(clk), 
        .rst(rst)
    );
    //Sign Extension
    sign_extend_immediate sign_ext(
        .in(instruction[7:0]),        // 8-bit input number
        .sign_extend_en(Sign_Ext_Flag),  // Control signal for enabling sign extension
        .out(Imm_ext_data)       // 16-bit output number
    );
    //Connect the RegisterFile Outputs, PC and IR to MUX1(18x1) with control signal `Sel_Bus_1_MUX`->5bits
    MUX18x1 MUX1(
        .s(Sel_Bus_1_MUX),
        .data_0(R0_out), 
        .data_1(R1_out), 
        .data_2(R2_out), 
        .data_3(R3_out), 
        .data_4(R4_out), 
        .data_5(R5_out), 
        .data_6(R6_out), 
        .data_7(R7_out), 
        .data_8(R8_out), 
        .data_9(R9_out), 
        .data_10(R10_out), 
        .data_11(R11_out), 
        .data_12(R12_out), 
        .data_13(R13_out), 
        .data_14(R14_out), 
        .data_15(R15_out), 
        .data_16(PC_addr), //OUTPUT FROM PC
        .data_17(Imm_ext_data), //Output from Sign Extended from Instruction Register
        .data_out(Bus_1) // Output to Bus_1
    ); 
    // Add RegY
    RegisterUnit Reg_Y (.data_out(Reg_Y_Out), .data_in(Bus_2), .load(Reg_Y_Ld), .clk(clk), .rst(rst));
    // Check if Reg_Y output is zero
    CheckZero Check_Zero_for_RegY(
        .Rp_data(Reg_Y_Out), // Input to Check Zero module from Reg_Y
        .Rp_zero(RF_Ry_Zero) // output to controller module
    );

    // Check ALU Zero FLAG
    D_flop Reg_Z(.data_out(alu_zero), .data_in(alu_zero_flag), .load(Reg_Z_Ld), .clk(clk), .rst(rst));
    // add ALU
    alu ALU_RISC(
        .alu_A(Reg_Y_Out), // One input to ALU from Reg_Y
        .alu_B(Bus_1), // One input to ALU from BUS
        .alu_OUT(alu_out), // Output from ALU
        .opcode(opcode), // Opcode to selet operation
        .alu_zero_flag(alu_zero_flag)
    ); 

    // Add Address Register
    AddressRegister Reg_A (.data_out(address), .data_in(Bus_2[7:0]), .load(Reg_A_Ld), .clk(clk), .rst(rst));
    
    // MUX 3X1 to select from  alu_output, bus_1, read_Data(from memory) 
    MUX3x1 MUX2(
        .s(Sel_Bus_2_MUX),
        .data_0(alu_out),
        .data_1(Bus_1),
        .data_2(mem_read_data),
        .data_out(Bus_2)
    );
    
endmodule

module alu(
    alu_A,
    alu_B,
    alu_OUT,
    opcode,
    alu_zero_flag
); 
    //datawidths
    parameter data_width = 16;
    parameter opcode_size = 4;
    
    //OPCODE Definition
    parameter ADD = 4'b0000; 
    parameter SUB = 4'b0001; 
    parameter AND = 4'b0010; 
    parameter OR = 4'b0011; 
    parameter XOR = 4'b0100; 
    parameter NOT = 4'b0101; 
    parameter SLA = 4'b0110; 
    parameter SRA = 4'b0111; 
    parameter LI = 4'b1000; 
    parameter LW = 4'b1001;
    parameter SW = 4'b1010; 
    parameter BIZ = 4'b1011;
    parameter BNZ = 4'b1100; 
    parameter JAL = 4'b1101;
    parameter JMP = 4'b1110; 
    parameter JR = 4'b1111; 
    
    // Input Output Declaration 
    input [data_width-1:0] alu_A; // Rs in 2's Complement
    input [data_width-1:0] alu_B; //Rt in 2's Complement
    input [opcode_size-1:0] opcode; // Opcode
    output [data_width-1:0] alu_OUT; //Rd in 2's Complenent
    output alu_zero_flag; // If output is 0

    // Reg Wire declation
    reg [data_width-1:0] alu_OUT;

    //Assign `alu_zero_flag` if alu_OUT is 0
    assign alu_zero_flag = ~|alu_OUT;

    // Choose what operation need to do
    always @(opcode or alu_A or alu_B) begin 
        case(opcode) 
            ADD:   alu_OUT = alu_A + alu_B;
            SUB:   alu_OUT = alu_A - alu_B;
            AND:   alu_OUT = alu_A & alu_B;
            OR:    alu_OUT = alu_A | alu_B;
            XOR:   alu_OUT = alu_A ^ alu_B;
            NOT:   alu_OUT = ~alu_B;
            SLA:   alu_OUT = alu_B << 1;
            SRA:   alu_OUT = alu_B >> 1;
            default:    alu_OUT = 16'b0;
        endcase
    end
endmodule

//16-bit 3x1 MUX
module MUX3x1(
    s,
    data_0,
    data_1,
    data_2,
    data_out
);
    parameter data_width = 16;
    parameter sel_width = 2;
    //input output declaration
    input [sel_width-1:0] s;
    input [data_width-1:0] data_0, data_1, data_2;
    output [data_width-1:0] data_out;

    //logic for 3x1 MUX
    assign data_out = (s==0)? data_0: (s==1)? data_1: (s==2)? data_2: 16'bx; 

endmodule

//16-bit 2x1 MUX
module MUX2x1(
    s,
    data_0,
    data_1,
    data_out
); 
    parameter data_width = 16;
    //input output declaration
    input s;
    input [data_width-1:0] data_0, data_1;
    output [data_width-1:0] data_out;

    // logic 2x1 MUX
    assign data_out = s ? data_0: data_1;        
endmodule

//16-bit 18x1 MUX
module MUX18x1(
    s,
    data_0, data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10, data_11, data_12, data_13, data_14, data_15, data_16, data_17,
    data_out
); 
    parameter data_width = 16;
    parameter sel_width = 5;
    //input output declaration
    input [sel_width-1:0] s;
    input [data_width-1:0] data_0, data_1, data_2, data_3, data_4, data_5, data_6, data_7, data_8, data_9, data_10, data_11, data_12, data_13, data_14, data_15, data_16, data_17;
    output [data_width-1:0] data_out;

    // logic 2x1 MUX
    assign data_out = (s==0)? data_0: (s==1)? data_1: (s==2)? data_2:(s==3)? data_3: (s==4)? data_4: (s==5)? data_5: (s==6)? data_6: (s==7)? data_7: (s==8)? data_8: (s==9)? data_9: (s==10)? data_10: (s==11)? data_11: (s==12)? data_12:  (s==13)? data_13:  (s==14)? data_14: (s==15)? data_15: (s==16)? data_16: (s==17)? data_17: 16'bx;      
endmodule

// Instruction Register for RISC Processor

module InstructionRegister(
    instr_in, 
    IR_ld, 
    instr_out, 
    clk, 
    rst
    );
    // instruction width parameter
    parameter instruction_width = 16;

    // Input output declaration 
    input [instruction_width-1:0] instr_in;
    input IR_ld;
    input clk;
    input rst;
    output reg [instruction_width-1:0] instr_out;

    // logic
    always @(posedge clk or negedge rst) begin 
        if (rst==0) begin 
            instr_out <= 0;
        end
        else if (IR_ld) begin 
            instr_out <= instr_in;
        end
    end

endmodule

// Design a Program Counter for RISC Processor
module ProgramCounter(
    ctr, 
    up, 
    clr, 
    load, 
    clk, 
    data_in
);
    // Parameter data_width
    parameter data_width = 16;
    // Input and Output
    input [data_width-1:0] data_in;
    input clr; // reset PC
    input load; // Load PC
    input clk; 
    input up; // Increment PC
    output [data_width-1:0]  ctr;
    // Declare register
    reg [data_width-1:0] ctr;

    always @(posedge clk or negedge clr) begin 
        if (clr==0) begin 
            ctr <= 0;
        end
        else if (load) begin 
            ctr <= data_in;
        end
        else if (up) begin 
            ctr <= ctr + 1;
        end
    end
endmodule


// Compute PC ctr given A = current address, B = Offset

module compute_offset(
    A, // PC Address
    B, // Offset
    new_A
);
    // Parameter
    parameter data_width = 16;

    input [data_width-1:0] A;
    input [data_width-1:0] B;
    output [data_width-1:0] new_A;

    // Logic
    assign new_A = A + B - 1;

endmodule

module AddressRegister(
    data_out, 
    data_in, 
    load, 
    clk, 
    rst
    );
    //address width  
    parameter addr_width = 8;
    output [addr_width-1:0] data_out;
    input [addr_width-1:0] data_in;
    input load, clk, rst;
    reg [addr_width-1:0] data_out;

    always @(posedge clk or negedge rst) begin 
        if (rst==0) begin 
            data_out <= 0;
        end
        else if (load) begin 
            data_out <= data_in;
        end
    end
endmodule

// Check if Register Rp is equal to zero

module CheckZero(
    Rp_data,
    Rp_zero
);
    // data_width
    parameter data_width = 16;
    //input output data declaration
    input [data_width-1:0] Rp_data;
    output Rp_zero;
    //Assign `alu_zero_flag` if alu_OUT is 0
    assign Rp_zero = ~|Rp_data;
endmodule

module D_flop(data_out, data_in, load, clk, rst);
    output data_out;
    input data_in;
    input load;
    input clk, rst;
    reg data_out;

    always @(posedge clk or negedge rst) begin 
        if (rst==0) begin 
            data_out <= 0;
        end
        else if (load==1) begin 
            data_out <= data_in;
        end
    end
endmodule

module RegisterUnit(data_out, data_in, load, clk, rst); 
    // data width
    parameter data_width = 16;

    // Input output declaration
    output [data_width-1:0] data_out;
    input [data_width-1:0] data_in;
    input load;
    input clk,rst;
    reg [data_width-1:0] data_out;

    always @(posedge clk or negedge rst) begin 
        if (rst==0) begin 
            data_out <= 0;
        end
        else if (load) begin 
            data_out <= data_in;
        end
    end
endmodule

module increment(
    inp, 
    out
);
    // address widht
    parameter addr_width = 8;

    //input output declaration 
    input [addr_width-1:0] inp;
    output [addr_width-1:0] out;

    // Logic 
    assign out = inp + 1;
    
endmodule

module decoder_4_to_16 (
    input wire [3:0] binary_in,  // 4-bit binary input
    output reg [15:0] dec_out    // 16-bit decoder output
);

    // Always block to implement combinational logic
    always @(*) begin
        // Initialize output to all zeros
        dec_out = 16'b0;
        
        // Set the appropriate output bit based on the input
        case (binary_in)
            4'b0000: dec_out[0] = 1'b1;
            4'b0001: dec_out[1] = 1'b1;
            4'b0010: dec_out[2] = 1'b1;
            4'b0011: dec_out[3] = 1'b1;
            4'b0100: dec_out[4] = 1'b1;
            4'b0101: dec_out[5] = 1'b1;
            4'b0110: dec_out[6] = 1'b1;
            4'b0111: dec_out[7] = 1'b1;
            4'b1000: dec_out[8] = 1'b1;
            4'b1001: dec_out[9] = 1'b1;
            4'b1010: dec_out[10] = 1'b1;
            4'b1011: dec_out[11] = 1'b1;
            4'b1100: dec_out[12] = 1'b1;
            4'b1101: dec_out[13] = 1'b1;
            4'b1110: dec_out[14] = 1'b1;
            4'b1111: dec_out[15] = 1'b1;
            default: dec_out = 16'b0;  // Default case for safety
        endcase
    end
endmodule

module sign_extend_immediate (
    input wire [7:0] in,        // 8-bit input number
    input wire sign_extend_en,  // Control signal for enabling sign extension
    output reg [15:0] out       // 16-bit output number
);

    always @(*) begin
        if (sign_extend_en) begin
            // If sign_extend_en is high, perform sign extension
            out = {{8{in[7]}}, in};  // Replicate the sign bit (in[7]) and concatenate with the input
        end else begin
            // If sign_extend_en is low, zero-extend the input
            out = {8'b0, in};  // Concatenate 8 zeros with the input
        end
    end

endmodule

//CHNAGES

/*
Merge states into one state

S_ex1 = S_dec_source2 + S_ex1
*/

module ControlUnit2(
    PC_Ld, PC_Inc, sel_PC_Offset_Update, // Control Signals for PC
    RF_W_Addr,  // Address to write into Register File
    IR_Ld, // Control Signals to Load Instruction Register
    Sign_Ext_Flag, // Control Signal for Sign Extension immediate
    Sel_Bus_1_MUX, // Control Bus_1 MUX1
    Reg_Y_Ld, // Control Signal for Reg_Y
    RF_Ry_Zero, //Output Flag to check operand in RegY is 0 or not
    instruction, //Output instrution bytes
    Sel_Bus_2_MUX, // Control Bus_2 MUX2
    Reg_A_Ld, // Control Signal to Load Reg_A (Address_Register)
    Reg_Z_Ld, // Control Signal to Load Reg_Z (Zero ALU Flag)
    alu_zero, //Output flag for alu_output is 0 or not
    D_wr, // Data Write Enable signal to read memory
    clk, rst);

    // Parameter declaration
    parameter data_width = 16; // Width of the data
    parameter addr_width = 8; // Address width for memory and Program Counter
    parameter RF_W_Addr_Width = 4; //Register file address width
    // Instruction Format
    // instruction = xxxx_xxxx_xxxx_xxxx / opcode destination src1 src2
    // Opcode = instruction[15:12], Destination = instruction[11:8], Source = instruction[7:0]/ src1 = instruction[7:4] and src2 = instruction[3:0]
    parameter opcode_size  = 4; // Size of the opcode
    parameter state_size = 4; // 2^4 states
    parameter source_size = 8; // Last 8 bits are either address source or src1 + src2
    parameter src1_size = 4;
    parameter src2_size = 4;
    parameter dest_size = 4;
    // Select buses widths
    parameter sel_bus_1_size = 5; //select control width
    parameter sel_bus_2_size = 2; // select control width
    
    // States definition 
    parameter S_idle = 0;
    parameter S_fet1 = 1;
    parameter S_fet2 = 2;
    parameter S_dec_source_1 = 3;
    parameter S_dec_source_2 = 4;
    parameter S_ex1 = 5;
    parameter S_rd1 = 6;
    parameter S_rd2 = 7;
    parameter S_wr1 = 8;
    parameter S_wr2 = 9;
    /*Need to add some more states for BIZ, BNZ, JAL, JR, JMP*/
    parameter S_halt = 10;

    //OPCODE Definition
    parameter ADD = 4'b0000; 
    parameter SUB = 4'b0001; 
    parameter AND = 4'b0010; 
    parameter OR = 4'b0011; 
    parameter XOR = 4'b0100; 
    parameter NOT = 4'b0101; 
    parameter SLA = 4'b0110; 
    parameter SRA = 4'b0111; 
    parameter LI = 4'b1000; 
    parameter LW = 4'b1001;
    parameter SW = 4'b1010; 
    parameter BIZ = 4'b1011;
    parameter BNZ = 4'b1100; 
    parameter JAL = 4'b1101;
    parameter JMP = 4'b1110; 
    parameter JR = 4'b1111; 
    parameter NO_INSTR = 4'bx;

    // Source and Destination Codes
    parameter R0 = 4'd0;
    parameter R1 = 4'd1;
    parameter R2 = 4'd2;
    parameter R3 = 4'd3;
    parameter R4 = 4'd4;
    parameter R5 = 4'd5;
    parameter R6 = 4'd6;
    parameter R7 = 4'd7;
    parameter R8 = 4'd8;
    parameter R9 = 4'd9;
    parameter R10 = 4'd10;
    parameter R11 = 4'd11;
    parameter R12 = 4'd12;
    parameter R13 = 4'd13;
    parameter R14 = 4'd14;
    parameter R15 = 4'd15;
    parameter RYis0 = 0;
    parameter RYis1 = 1;

    // Input output declaration 
    output PC_Ld;
    output PC_Inc;
    output sel_PC_Offset_Update;
    output [RF_W_Addr_Width-1:0] RF_W_Addr;
    output IR_Ld;
    output Sign_Ext_Flag;
    output [sel_bus_1_size-1:0] Sel_Bus_1_MUX;
    output Reg_Y_Ld;
    output [sel_bus_2_size-1:0] Sel_Bus_2_MUX;
    output Reg_A_Ld;
    output Reg_Z_Ld;
    output D_wr;

    input RF_Ry_Zero;
    input [data_width-1:0] instruction;
    input alu_zero;
    input clk, rst;

    // Reg declaration 
    reg PC_Ld;
    reg PC_Inc;
    reg sel_PC_Offset_Update;
    reg [RF_W_Addr_Width-1:0] RF_W_Addr;
    reg IR_Ld;
    reg Sign_Ext_Flag;
    //reg [sel_bus_1_size-1:0] Sel_Bus_1_MUX;
    reg Reg_Y_Ld;
    //reg [sel_bus_2_size-1:0] Sel_Bus_2_MUX;
    reg Reg_A_Ld;
    reg Reg_Z_Ld;
    reg D_wr;



    // Internal Reg declaration
    // Select Registers for controlling the BUSES
    reg Sel_R0;
    reg Sel_R1;
    reg Sel_R2;
    reg Sel_R3;
    reg Sel_R4;
    reg Sel_R5;
    reg Sel_R6;
    reg Sel_R7;
    reg Sel_R8;
    reg Sel_R9;
    reg Sel_R10;
    reg Sel_R11;
    reg Sel_R12;
    reg Sel_R13;
    reg Sel_R14;
    reg Sel_R15;
    reg Sel_PC;
    reg Sel_Sign_Ext;
    reg Sel_ALU, Sel_Bus_1, Sel_Mem;
    reg [state_size-1:0] state, next_state;
    reg err_flag; // add an error flag as well
    
    // wires for individualizing instrution parts
    wire [opcode_size-1:0] opcode = instruction[15:12];
    wire [dest_size-1:0] destination = instruction[11:8]; // Destination register name
    wire [source_size-1:0] source = instruction[7:0]; // source is last 7 bit  can be source 1 + source 2
    wire [src1_size-1:0] source1 = instruction[7:4]; // src1 or Operand 1
    wire [src2_size-1:0] source2 = instruction[3:0]; // Src2 or OPerand 2
    

    //Mux Selectors
    assign Sel_Bus_1_MUX[sel_bus_1_size-1:0] = Sel_R0 ? 0 :
                                           Sel_R1 ? 1 :
                                           Sel_R2 ? 2 :
                                           Sel_R3 ? 3 :
                                           Sel_R4 ? 4 :
                                           Sel_R5 ? 5 :
                                           Sel_R6 ? 6 :
                                           Sel_R7 ? 7 :
                                           Sel_R8 ? 8 :
                                           Sel_R9 ? 9 :
                                           Sel_R10 ? 10 :
                                           Sel_R11 ? 11 :
                                           Sel_R12 ? 12 :
                                           Sel_R13 ? 13 :
                                           Sel_R14 ? 14 :
                                           Sel_R15 ? 15 :
                                           Sel_PC ? 16:
                                           Sel_Sign_Ext ? 17:
                                           5'dx;  // Default value if none of the conditions are met

    assign Sel_Bus_2_MUX[sel_bus_2_size-1:0] = Sel_ALU ? 0 :
                                            Sel_Bus_1 ? 1:
                                            Sel_Mem ? 2:
                                            2'bx; // default value

    
    always @(posedge clk or negedge rst) begin: State_transitions 
        if (rst==0) begin 
            state <= S_idle;
        end 
        else begin 
            state <= next_state;
        end
    end

    always @(state or opcode or source or destination or alu_zero) begin: Output_and_Next_State
        Sel_R0 = 0;
        Sel_R1 = 0;
        Sel_R2 = 0;
        Sel_R3 = 0;
        Sel_R4 = 0;
        Sel_R5 = 0;
        Sel_R6 = 0;
        Sel_R7 = 0;
        Sel_R8 = 0;
        Sel_R9 = 0;
        Sel_R10 = 0;
        Sel_R11 = 0;
        Sel_R12 = 0;
        Sel_R13 = 0;
        Sel_R14 = 0;
        Sel_R15 = 0;
        Sel_PC = 0;
        Sel_Sign_Ext = 0;
        RF_W_Addr = 4'bx; // makes all dec_out = 16'b0 i.e. all Load_Ri = 0

        PC_Ld = 0;
        PC_Inc = 0;
        IR_Ld = 0;
        Sign_Ext_Flag = 0;
        Reg_Y_Ld = 0;
        Reg_A_Ld = 0;
        Reg_Z_Ld = 0;
        D_wr = 0;

        Sel_ALU = 0;
        Sel_Bus_1 = 0;
        Sel_Mem = 0;

        err_flag = 0; // Used for De-bug in simulation
        next_state = state;

        case (state) 
            S_idle: begin //State entered after reset is asserted
                        next_state = S_fet1;
            end
            S_fet1: begin  // Load Address register with the contents of PC
                        next_state = S_fet2;
                        Sel_PC = 1;
                        Sel_Bus_1 = 1;
                        Reg_A_Ld = 1;
            end
            S_fet2: begin // 1. Load the instruction register with the word addressed by the address register, 2. Increent PC to poin to the next location of memory
                        next_state = S_dec_source_1;
                        Sel_Mem = 1;
                        IR_Ld = 1;
                        PC_Inc = 1;
            end
            S_dec_source_1: begin
                        case (opcode) 
                            ADD, SUB, AND, OR, XOR: begin 
                                next_state = S_ex1; // get the 2nd operand as well ans store the result
                                //$display("Entering into ADD, SUB, AND, OR: Before Case ");
                                // $display("instruction = %h", instruction);
                                // $display("source1 = %h", source1);
                                // $display("source2 = %h", source2);
                                // $display("destination = %h", destination);
                                // $display("opcode = %h", opcode);
                                // $display("Sel_5 = %b", Sel_R5);
                                case (source1) // Load the operand to Reg_Y from given register of register file
                                    R0: Sel_R0 = 1;
                                    R1: Sel_R1 = 1;
                                    R2: Sel_R2 = 1;
                                    R3: Sel_R3 = 1;
                                    R4: Sel_R4 = 1;
                                    R5: Sel_R5 = 1;
                                    R6: Sel_R6 = 1;
                                    R7: Sel_R7 = 1;
                                    R8: Sel_R8 = 1;
                                    R9: Sel_R9 = 1;
                                    R10: Sel_R10 = 1;
                                    R11: Sel_R11 = 1;
                                    R12: Sel_R12 = 1;
                                    R13: Sel_R13 = 1;
                                    R14: Sel_R14 = 1;
                                    R15: Sel_R15 = 1;
                                    default err_flag = 1;
                                endcase
                                // $display("After case");
                                // $display("instruction = %h", instruction);
                                // $display("source1 = %h", source1);
                                // $display("source2 = %h", source2);
                                // $display("destination = %h", destination);
                                // $display("opcode = %h", opcode);
                                // $display("Sel_5 = %b", Sel_R5);
                                Sel_Bus_1 = 1;
                                Reg_Y_Ld = 1;
                            end // ADD, SUB, AND, OR, XOR

                            NOT, SLA, SRA: begin 
                                next_state = S_fet1;
                                case (source1) // Load the Operand to ALU insted of Reg Y from given register
                                    R0: Sel_R0 = 1;
                                    R1: Sel_R1 = 1;
                                    R2: Sel_R2 = 1;
                                    R3: Sel_R3 = 1;
                                    R4: Sel_R4 = 1;
                                    R5: Sel_R5 = 1;
                                    R6: Sel_R6 = 1;
                                    R7: Sel_R7 = 1;
                                    R8: Sel_R8 = 1;
                                    R9: Sel_R9 = 1;
                                    R10: Sel_R10 = 1;
                                    R11: Sel_R11 = 1;
                                    R12: Sel_R12 = 1;
                                    R13: Sel_R13 = 1;
                                    R14: Sel_R14 = 1;
                                    R15: Sel_R15 = 1;
                                    default err_flag = 1; 
                                endcase
                                Reg_Z_Ld = 1;
                                Sel_ALU = 1;
                                // Save the alu result to register in the register file again (address given in `destination`)
                                RF_W_Addr = destination;
                            end // NOT, SRA, SLA

                            LI: begin
                                next_state = S_fet1;
                                Sign_Ext_Flag = 1; // Get the sign extension
                                Sel_Sign_Ext = 1; // pass to bus_1
                                Sel_Bus_1 = 1; // pass the sign extended data to bus_2
                                RF_W_Addr = destination; //save the result to destination register in the register file
                            end // LI

                            LW: begin 
                                next_state = S_rd1; // next state to load the memory data to Register
                                Sign_Ext_Flag = 0; // Get memory address for obtaining data without sign extension immediate
                                Sel_Sign_Ext = 1; // get the mem address to the bus 1
                                Sel_Bus_1 = 1; // get the mem address to Bus 2
                                Reg_A_Ld = 1; // Load the Address register with the mem address
                            end // LW

                            SW: begin 
                                next_state = S_wr1; // Next State to load the reg to the memory
                                Sign_Ext_Flag = 0; // Get the memory address for the obtaining data without sign extension immediate
                                Sel_Sign_Ext = 1; // get the mem address to Bus 1
                                Sel_Bus_1 = 1; // get the mem address to Bus 2
                                Reg_A_Ld = 1; // Load the address register with the mem address
                            end // SW

                            BIZ: begin 
                                next_state = S_fet1; // Since BIZ completed in this statement it self
                                case (destination) // Load the operand to Reg_Y from given register of register file
                                    R0: Sel_R0 = 1;
                                    R1: Sel_R1 = 1;
                                    R2: Sel_R2 = 1;
                                    R3: Sel_R3 = 1;
                                    R4: Sel_R4 = 1;
                                    R5: Sel_R5 = 1;
                                    R6: Sel_R6 = 1;
                                    R7: Sel_R7 = 1;
                                    R8: Sel_R8 = 1;
                                    R9: Sel_R9 = 1;
                                    R10: Sel_R10 = 1;
                                    R11: Sel_R11 = 1;
                                    R12: Sel_R12 = 1;
                                    R13: Sel_R13 = 1;
                                    R14: Sel_R14 = 1;
                                    R15: Sel_R15 = 1;
                                    default err_flag = 1;
                                endcase
                                Sel_Bus_1 = 1; // tranfer the reg data to bus2
                                Reg_Y_Ld = 1; // Load the reg data to Reg_Y to check if its 0 or not
                                // This will give you `RF_Ry_Zero` value can be 0 or 1
                                case(RF_Ry_Zero) 
                                    RYis0: begin 
                                        sel_PC_Offset_Update = 0;
                                        PC_Ld = 0;
                                    end 
                                    RYis1: begin
                                        sel_PC_Offset_Update = 1;
                                        PC_Ld = 1; 
                                    end
                                endcase
                            end // BIZ

                            BNZ: begin 
                                next_state = S_fet1; // Since BNZ completed in this statement it self
                                case (destination) // Load the operand to Reg_Y from given register of register file
                                    R0: Sel_R0 = 1;
                                    R1: Sel_R1 = 1;
                                    R2: Sel_R2 = 1;
                                    R3: Sel_R3 = 1;
                                    R4: Sel_R4 = 1;
                                    R5: Sel_R5 = 1;
                                    R6: Sel_R6 = 1;
                                    R7: Sel_R7 = 1;
                                    R8: Sel_R8 = 1;
                                    R9: Sel_R9 = 1;
                                    R10: Sel_R10 = 1;
                                    R11: Sel_R11 = 1;
                                    R12: Sel_R12 = 1;
                                    R13: Sel_R13 = 1;
                                    R14: Sel_R14 = 1;
                                    R15: Sel_R15 = 1;
                                    default err_flag = 1;
                                endcase
                                Sel_Bus_1 = 1; // tranfer the reg data to bus2
                                Reg_Y_Ld = 1; // Load the reg data to Reg_Y to check if its 0 or not
                                // This will give you `RF_Ry_Zero` value can be 0 or 1
                                case(RF_Ry_Zero) 
                                    RYis0: begin // If RF_Ry_Zero = 0 
                                        sel_PC_Offset_Update = 1;
                                        PC_Ld = 1;
                                    end 
                                    RYis1: begin
                                        sel_PC_Offset_Update = 0;
                                        PC_Ld = 0; 
                                    end
                                endcase
                            end // BNZ

                            JAL: begin
                                // Rd = PC + 1
                                next_state = S_fet1; // JAL complete in this state
                                Sel_PC = 1; // Get the PC + 1 from the PC on Bus 1
                                Sel_Bus_1 = 1; // Get the PC+1 on Bus 2
                                RF_W_Addr = destination; // load PC + 1 from Bus_2 to the destination register
                                // Need to update PC to PC + offset (PC is already PC+1)
                                sel_PC_Offset_Update = 1; // select to load the new address (i.e. PC + Offset) to PC
                                PC_Ld = 1; // Load PC with new data
                            end // JAL

                            JMP: begin 
                                next_state = S_fet1;
                                // PC = PC + Offset (Since PC is already incremented to PC + 1)
                                sel_PC_Offset_Update = 1;
                                PC_Ld = 1;
                            end // JR

                            JR: begin 
                                next_state = S_fet1;
                                // PC = Rs (source1)
                                case (source1) // Load the operand to Reg_Y from given register of register file
                                    R0: Sel_R0 = 1;
                                    R1: Sel_R1 = 1;
                                    R2: Sel_R2 = 1;
                                    R3: Sel_R3 = 1;
                                    R4: Sel_R4 = 1;
                                    R5: Sel_R5 = 1;
                                    R6: Sel_R6 = 1;
                                    R7: Sel_R7 = 1;
                                    R8: Sel_R8 = 1;
                                    R9: Sel_R9 = 1;
                                    R10: Sel_R10 = 1;
                                    R11: Sel_R11 = 1;
                                    R12: Sel_R12 = 1;
                                    R13: Sel_R13 = 1;
                                    R14: Sel_R14 = 1;
                                    R15: Sel_R15 = 1;
                                    default err_flag = 1;
                                endcase
                                Sel_Bus_1 = 1; // tranfer the reg data to bus2
                                sel_PC_Offset_Update = 0; // Load the Bus data
                                PC_Ld = 1; // Update the new data from Bus to PC;
                            end 


                        endcase
            end
            S_ex1: begin 
                //Next state will be execute 
                next_state = S_fet1; // Fetch after execution is complete
                // read the seond operand value for ADD, SUB, AND, OR, XOR
                case (source2) // Load the operand to BUS_1 to ALU from given register of register file
                    R0: Sel_R0 = 1;
                    R1: Sel_R1 = 1;
                    R2: Sel_R2 = 1;
                    R3: Sel_R3 = 1;
                    R4: Sel_R4 = 1;
                    R5: Sel_R5 = 1;
                    R6: Sel_R6 = 1;
                    R7: Sel_R7 = 1;
                    R8: Sel_R8 = 1;
                    R9: Sel_R9 = 1;
                    R10: Sel_R10 = 1;
                    R11: Sel_R11 = 1;
                    R12: Sel_R12 = 1;
                    R13: Sel_R13 = 1;
                    R14: Sel_R14 = 1;
                    R15: Sel_R15 = 1;
                    default err_flag = 1;
                endcase
                Reg_Z_Ld = 1; // Enable Zero flag for ALU output
                Sel_ALU = 1; // Send ALU result to BUS 2
                // Now save the result to destinatination register
                RF_W_Addr = destination;
            end
            // S_ex1: begin 
            //     next_state = S_fet1; // Fetch after execution is complete
            //     Reg_Z_Ld = 1; // Enable Zero flag for ALU output
            //     Sel_ALU = 1; // Send ALU result to BUS 2
            //     // Now save the result to destinatination register
            //     RF_W_Addr = destination;
            // end
            S_rd1: begin 
                next_state = S_fet1; // Since load operation is complete
                Sel_Mem = 1; // Get data to Bus 2 from the memory
                RF_W_Addr = destination; //save the result to destination register in the register file
            end
            S_wr1: begin 
                next_state = S_fet1; // Since store operation is complete
                D_wr = 1; // Enable write data present on Bus 1 to the memory
                case (destination) // Load the Operand to BUS_1 insted which is connected to memory
                    R0: Sel_R0 = 1;
                    R1: Sel_R1 = 1;
                    R2: Sel_R2 = 1;
                    R3: Sel_R3 = 1;
                    R4: Sel_R4 = 1;
                    R5: Sel_R5 = 1;
                    R6: Sel_R6 = 1;
                    R7: Sel_R7 = 1;
                    R8: Sel_R8 = 1;
                    R9: Sel_R9 = 1;
                    R10: Sel_R10 = 1;
                    R11: Sel_R11 = 1;
                    R12: Sel_R12 = 1;
                    R13: Sel_R13 = 1;
                    R14: Sel_R14 = 1;
                    R15: Sel_R15 = 1;
                    default err_flag = 1; 
                endcase
                
            end
            default: begin 
                next_state = S_idle;
            end
        endcase
    end
endmodule

// Designing the main memory for RISC PRocessor
// Size of the memory 256x16 i.e. 256 locations, from 0x00/00000000 to 0xFF/11111111
// Each location 16 bit

module main_memory(
    W_data,
    R_data,
    wr,
    addr,
    clk
); 
    
    parameter depth = 256; // Depth of the memory
    parameter addr_width = 8; // Address width
    parameter data_width = 16; // Parameter widht of the data

    // input and output declaration
    input [data_width-1:0] W_data; // data_in
    input [addr_width-1:0] addr; // address
    input wr; // write
    input clk;
    output [data_width-1:0] R_data; // data out

    // Wire reg declaration 
    //reg [data_width-1:0] R_data;
    //reg [data_width-1:0] W_data;
    
    // Declare memory reggister
    reg [data_width-1:0] memory256x16 [depth-1:0];

    // insert the instruction and data to the memory
    initial begin 
        memory256x16[0] = 16'b1001_0101_1100_1001; // LW 5 201
        memory256x16[1] = 16'b1001_0110_1100_1010; // LW 6 202
        memory256x16[2] = 16'b0000_0111_0101_0110; // ADD 7 5 6
        memory256x16[3] = 16'b1010_0111_1100_1011; // SW 7 203
        memory256x16[4] = 16'b1000_1000_1111_1010; // LI 8 250
        memory256x16[5] = 16'b0001_0100_1000_0101; // SUB 4 8 5
        memory256x16[6] = 16'b1010_0100_1100_1100; //SW 4 204
        memory256x16[7] = 16'b0111_0011_0111_0000; // SRA 3 7
        memory256x16[8] = 16'b0100_0010_0011_0100; // XOR 2 3 4
        memory256x16[9] = 16'b1010_0010_1100_1101; // SW 2 205
        memory256x16[201] = 16'h1111; // Initial value for location 201
        memory256x16[202] = 16'h2222; // Initial value for location 202
    end

    
    assign R_data = memory256x16[addr];
    //Writing data
    always @(posedge clk) begin
        if (wr) begin 
            memory256x16[addr] <= W_data;
        end
    end


endmodule