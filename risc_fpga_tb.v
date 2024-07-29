
`include "risc_fpga_full_program.v"


module risc_fpga_tb;

    // Inputs
    reg clk;
    reg [15:0] reset;
    wire [7:0] disp_an_o;
    wire [7:0] disp_seg_o;

    always begin
        #5 clk = ~clk; // Toggle clock every 5 ticks // 100 MhZ clok
    end


    initial begin
        $dumpfile("risc_fpga_tb.vcd");
        $dumpvars(0, risc_fpga_tb);
        // $monitor("Mem[203]=%h, Mem[204]=%h, Mem[205]=%h", machine.MU.memory256x16[203], machine.MU.memory256x16[204], machine.MU.memory256x16[205]);
        //$monitor("$time = %g, clk=%b, instruction_in_CU: %h , Source1 [Should be 5]: %h, Source2 [Should be 6]: %h, Sel_Bus_1_MUX [Should be 5 as well]: %d", $time, clk, machine.CU.instruction, machine.CU.source1, machine.CU.source1, machine.CU.Sel_Bus_1_MUX);

        //monitor the states
        //$monitor("instruction=%h, current_state=%d, next_state=%d", machine.CU.instruction, machine.CU.state, machine.CU.next_state);
        clk = 0;
        risc_fpga_im.select = 0;
        risc_fpga_im.segment.select = 0;
        reset = 0;
        
        #10
        reset = 1;
        #1_000_000_000; // 
        $finish; 
        
    end
    
    // Instantiate the risc machine
    risc_fpga_ risc_fpga_im(
        .SW(reset), // used for 
        .clk_i(clk),
        .disp_an_o(disp_an_o), // anodes to select display Should always be 1110000
        .disp_seg_o(disp_seg_o) // Cathodes to select segments
);

endmodule
