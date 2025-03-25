`timescale 1ns / 1ps

module led_driver_wrapper( led_value, leds, write_to_leds, clk, reset );
	input [7:0] led_value;
	output reg [7:0] leds;
	input write_to_leds;
	input clk;
	input reset;
	
	always @(posedge clk or posedge reset) begin
		if(reset) leds <= 0;
		else if(write_to_leds) leds <= led_value;
	end

endmodule
