`include "risc_machine2.v"
`include "led_segment.v"

module risc_fpga(
    input [15:0] SW,
    input clk_i,
    output reg [7:0] disp_an_o, // anodes to select display Should always be 1110000
    output reg [7:0] disp_seg_o, // Cathodes to select segment
    //output reg [15:0] LED // LED to show for clock
); 
    wire [15:0] mem_loc_201;
    wire [15:0] mem_loc_202;
    wire [15:0] mem_loc_203;
    wire [15:0] mem_loc_204;
    wire [15:0] mem_loc_205;
    wire [15:0] SW; // this is the reset option given from FPGA switch (SW[0] )
    // Get the values of the memory location from RISC FPGA
    risc_machine machine(.clk(clk_i), .rst(SW[0]));

    // assing the memory values to the memory location
    assign mem_loc_201 = machine.MU.memory256x16[201];
    assign mem_loc_202 = machine.MU.memory256x16[202];
    assign mem_loc_203 = machine.MU.memory256x16[203];
    assign mem_loc_204 = machine.MU.memory256x16[204];
    assign mem_loc_205 = machine.MU.memory256x16[205];

    // send mem_loc one be one to bin to bcd
    //make a 0.25Hz clock input
    reg [16:0] counter1; // 17-bit counter to divide 100MHDZ Clock into 1KHz
    reg [2:0] select1; // Selector for the mux (3 bits because only need 5) 
    always @(posedge clk_i) begin 
        if (counter1 != 200_000_000) begin 
            counter1 <= counter1 + 1;
        end
        else begin 
            counter1 <= 0;
            //LED <= ~LED;
            select1 <= (select1 < 4)? select1+1:0; //loop through 0 to 4
        end
    end

    // MUX to send values
    reg [15:0] mem_to_led;

    //MUX to send value according to select1 (1HZ)
    always @(*) begin
        // use case statement 
        case (select1)
            3'd0: begin 
                mem_to_led = mem_loc_201
            end
            3'd1: begin
                mem_to_led = mem_loc_202
            end
            3'd2: begin
                mem_to_led = mem_loc_203
            end
            3'd3: begin
                mem_to_led = mem_loc_204
            end
            3'd4: begin
                mem_to_led = mem_loc_205
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
        cathodes(disp_seg_o), // Cathodes to select segment
        //output reg [15:0] LED // LED to show for clock
    );



endmodule