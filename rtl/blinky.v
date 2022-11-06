module blinky #()
(
	input CLK_48,

	output LED_A,

	inout PMOD_A1, PMOD_A2, PMOD_A3, PMOD_A4,
		PMOD_A7, PMOD_A8, PMOD_A9, PMOD_A10,

);

	wire clk = CLK_48;

	reg [26:0] counter = 0;

	assign LED_A = ~counter[26];

	always @(posedge clk) begin

		counter <= counter + 1;

	end

endmodule
