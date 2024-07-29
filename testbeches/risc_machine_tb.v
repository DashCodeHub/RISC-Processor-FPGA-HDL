`include "risc_machine.v"


module risc_machine_tb;

    // Inputs
    reg clk;
    reg rst;

    always begin
        #5 clk = ~clk; // Toggle clock every 5 ticks
    end


    initial begin
        $dumpfile("risc_machine_tb.vcd");
        $dumpvars(0, risc_machine_tb);
        //$monitor("Mem[203]=%h, Mem[204]=%h, Mem[205]=%h", machine.MU.memory256x16[203], machine.MU.memory256x16[204], machine.MU.memory256x16[205]);
        //$monitor("$time = %g, clk=%b, instruction_in_CU: %h , Source1 [Should be 5]: %h, Source2 [Should be 6]: %h, Sel_Bus_1_MUX [Should be 5 as well]: %d", $time, clk, machine.CU.instruction, machine.CU.source1, machine.CU.source1, machine.CU.Sel_Bus_1_MUX);

        //monitor the states
        $monitor("instruction=%h, current_state=%d, next_state=%d", machine.CU.instruction, machine.CU.state, machine.CU.next_state);
        clk = 0;
        rst = 0;
        #10
        rst = 1;
        #300;
        $finish; 
        
    end
    
    // Instantiate the risc machine
    risc_machine machine(clk, rst);

endmodule


//$display("which register is selected [Should be 5]: %d", Sel_Bus_1_MUX)
