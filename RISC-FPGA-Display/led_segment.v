`timescale 1ns / 1ps

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
    output reg [7:0] cathodes, // Cathodes to select segment
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
        if (counter != 50_000) begin // Make is 50000 when running in fpga
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
